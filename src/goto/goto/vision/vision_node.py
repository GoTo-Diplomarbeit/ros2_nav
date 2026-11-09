import os
import time
from datetime import datetime
from enum import Enum

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper
from goto.state.mission_state import State
from goto.vision.image_recognition import ImageRecognition

# minimum box area of person
# aka how close the person is to the camera
MIN_PERSON_AREA = 28000

# box width and height where person is supposed to stand in
# box is always centered
BOX_WIDTH = 250
BOX_HEIGHT = 465
BOX_COLOR = (0, 255, 0)  # green (BGR)

# a face belongs to a person if the face center lies inside the person box
# and within this upper fraction of the person box (the head is at the top)
FACE_TOP_FRACTION = 0.4

# extra margin around the person box when cropping the saved photo (fraction of box width / height)
CROP_PADDING = 0.1

COUNTDOWN_S = 5.0
PERSON_TIMEOUT_S = 10.0  # how long to wait for a person after the countdown

# inside docker this is /root/go2_detections, bind-mounted to the host's ~/go2_detections
SAVE_DIR = os.path.expanduser("~/go2_detections")

PERSON_MODEL_PATH = os.path.join("weights", "yolov8n.pt")
FACE_MODEL_URL = "https://github.com/akanametov/yolo-face/releases/download/1.0.0/yolov8n-face.pt"

FAILED_RESULT = ""  # published on /vision/picture_result when no photo was taken


# maybe TODO: photo confirmation in the app


class CaptureState(Enum):
    IDLE = "idle"
    COUNTDOWN = "countdown"  # person has to stay in position until the countdown ends
    WAITING_FOR_PERSON = "waiting_for_person"  # countdown over, photo is taken on the next valid frame


# --
# -- Box helpers. A box is (x1, y1, x2, y2, ...), extra entries (e.g. confidence) are ignored
# --


def box_area(box) -> float:
    x1, y1, x2, y2 = box[:4]
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def is_box_inside(box, rectangle) -> bool:
    box_x1, box_y1, box_x2, box_y2 = box[:4]
    rect_x1, rect_y1, rect_x2, rect_y2 = rectangle
    return box_x1 >= rect_x1 and box_y1 >= rect_y1 and box_x2 <= rect_x2 and box_y2 <= rect_y2


def target_rectangle_for(frame):
    """The centered rectangle (green box) the person has to stand in."""
    height, width = frame.shape[:2]
    x1 = (width - BOX_WIDTH) // 2
    y1 = (height - BOX_HEIGHT) // 2
    return x1, y1, x1 + BOX_WIDTH, y1 + BOX_HEIGHT


def face_for_person(person, faces):
    """The most confident face whose center lies in the head area (upper part) of the person box, or None."""
    px1, py1, px2, py2 = person[:4]
    head_limit = py1 + (py2 - py1) * FACE_TOP_FRACTION

    def in_head_area(face):
        center_x = (face[0] + face[2]) / 2
        center_y = (face[1] + face[3]) / 2
        return px1 <= center_x <= px2 and py1 <= center_y <= head_limit

    matching_faces = [face for face in faces if in_head_area(face)]
    # max() keeps the first face on equal confidence, same as a strict ">" comparison
    return max(matching_faces, key=lambda face: face[4], default=None)


def select_person(people, faces):
    """The closest person (largest box) that has a matching face -> (person, face), or (None, None)."""
    for person in sorted(people, key=box_area, reverse=True):
        face = face_for_person(person, faces)
        if face is not None:
            return person, face
    return None, None


def crop_to_person(frame, person, target_rectangle):
    """
    Crop the frame to the person box plus CROP_PADDING, clamped to the image borders and to the
    target rectangle (so the padding never reaches people standing next to the rectangle).
    """
    height, width = frame.shape[:2]
    rect_x1, rect_y1, rect_x2, rect_y2 = target_rectangle
    x1, y1, x2, y2 = person[:4]
    pad_x = (x2 - x1) * CROP_PADDING
    pad_y = (y2 - y1) * CROP_PADDING
    x1 = max(0, rect_x1, int(x1 - pad_x))
    y1 = max(0, rect_y1, int(y1 - pad_y))
    x2 = min(width, rect_x2, int(x2 + pad_x))
    y2 = min(height, rect_y2, int(y2 + pad_y))
    return frame[y1:y2, x1:x2]


class VisionNode(NodeWrapper):
    def __init__(self):
        super().__init__("vision", self._setup_channels)

        # todo for later: add more states when the other states get implemented
        self.active_states = frozenset({State.REGISTER_PERSON})

    def _setup_channels(self):
        self._logger = self.node.get_logger()

        os.makedirs(os.path.dirname(PERSON_MODEL_PATH), exist_ok=True)
        self._person_recognition = ImageRecognition(PERSON_MODEL_PATH)
        self._face_recognition = ImageRecognition(FACE_MODEL_URL)

        self._bridge = CvBridge()
        self._busy = False  # skip incoming frames while the last one is still being processed

        os.makedirs(SAVE_DIR, exist_ok=True)

        # capture: IDLE -> COUNTDOWN -> WAITING_FOR_PERSON -> IDLE
        self._capture_state = CaptureState.IDLE
        self._countdown_end = 0.0
        self._wait_end = 0.0
        self._last_logged_second = None

        self._image_subscription = self.node.create_subscription(
            Image,
            "/d455/image",
            self._image_callback,
            1,  # small queue so you always process a recent frame
        )

        # annotated image (faces + people + target rectangle) to view in rviz / rqt_image_view
        self._debug_pub = self.node.create_publisher(Image, "/vision/person_recognition", 1)

        # Trigger from the other node, and result back to it TODO::: get from the correct node
        self._trigger_sub = self.node.create_subscription(Empty, "/vision/take_picture", self._on_trigger, 10)
        self._result_pub = self.node.create_publisher(String, "/vision/picture_result", 10)

    # --
    # -- Capture control
    # --

    # this starts the photo mode
    def _on_trigger(self, _msg: Empty) -> None:
        if not self.is_active():
            self._logger.warn("Cannot capture, state is not active")
            self._publish_result(FAILED_RESULT)
            return
        if self._capture_state != CaptureState.IDLE:
            self._logger.warn("Capture already in progress, ignoring trigger")
            return

        self._capture_state = CaptureState.COUNTDOWN
        self._countdown_end = time.monotonic() + COUNTDOWN_S
        self._last_logged_second = None
        self._logger.info(f"Trying to take a picture in {COUNTDOWN_S:.0f} s")

    # mission state left REGISTER_PERSON (or went unknown): abort a running capture,
    # otherwise it would resume with expired timers the next time the state is entered
    def on_state_exit(self, state: State) -> None:
        if self._capture_state != CaptureState.IDLE:
            self._logger.warn(f"Mission state left {state.value}, capture cancelled")
            self._cancel_capture()

    # reset the capture and tell the requester that it failed
    def _cancel_capture(self) -> None:
        self._capture_state = CaptureState.IDLE
        self._publish_result(FAILED_RESULT)

    def _publish_result(self, path: str) -> None:
        self._result_pub.publish(String(data=path))

    # --
    # -- Per-frame processing
    # --

    def _image_callback(self, image: Image) -> None:
        # only activate when the current global state is REGISTER_PERSON
        if self._busy or not self.is_active():
            return
        self._busy = True
        try:
            # run both models on the same clean frame (never on an image with boxes drawn on it)
            frame = self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            faces = self._face_recognition.get_results(frame)
            people = self._person_recognition.get_results(frame)
            target_rectangle = target_rectangle_for(frame)

            self._publish_debug_image(frame, target_rectangle, image.header)
            self._update_capture(frame, people, faces, target_rectangle)
        finally:
            self._busy = False

    # draw face boxes, person boxes and the target rectangle onto one copy of the frame and publish it
    def _publish_debug_image(self, frame, target_rectangle, header) -> None:
        annotated = self._face_recognition.draw(frame.copy())
        annotated = self._person_recognition.draw(annotated)
        x1, y1, x2, y2 = target_rectangle
        cv2.rectangle(annotated, (x1, y1), (x2, y2), BOX_COLOR, 2)

        out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = header
        self._debug_pub.publish(out)

    def _update_capture(self, frame, people, faces, target_rectangle) -> None:
        now = time.monotonic()

        if self._capture_state == CaptureState.COUNTDOWN:
            self._update_countdown(now, people, faces, target_rectangle)

        # not elif: when the countdown ends, the photo can already be taken on the same frame
        if self._capture_state == CaptureState.WAITING_FOR_PERSON:
            self._try_capture(now, frame, people, faces, target_rectangle)

    def _update_countdown(self, now, people, faces, target_rectangle) -> None:
        # return to idle state if the person is not standing in front of the camera properly
        # TODO: a single bad frame (YOLO detection flicker) cancels the whole countdown -
        #  add a grace period (e.g. only cancel after N bad frames in a row / ~0.5 s)
        if self._get_visible_person(people, faces, target_rectangle) is None:
            self._logger.warn("Countdown cancelled!")
            self._cancel_capture()
            return

        remaining = self._countdown_end - now
        if remaining <= 0:
            self._capture_state = CaptureState.WAITING_FOR_PERSON
            self._wait_end = now + PERSON_TIMEOUT_S
            return

        # log every second only once
        second = int(remaining) + 1
        if second != self._last_logged_second:
            self._last_logged_second = second
            self._logger.info(f"{second}...")

    def _try_capture(self, now, frame, people, faces, target_rectangle) -> None:
        person = self._get_visible_person(people, faces, target_rectangle)
        if person is not None:
            self._save_photo(frame, person, target_rectangle)
            self._capture_state = CaptureState.IDLE
        elif now > self._wait_end:
            self._logger.warn("No person and face inside the target box, capture cancelled")
            self._cancel_capture()

    # --
    # -- Person check and saving
    # --

    def _get_visible_person(self, people, faces, target_rectangle):
        """
        Check that the person inside the target rectangle is properly visible.
        Returns the person box, or None (with a warning why) if no photo can be taken.
        """
        if not people and not faces:
            return self._reject("No person detected.")
        if people and not faces:
            return self._reject("Either too close or the face is obstructed")
        if not people and faces:
            return self._reject("The person must face the camera and the entire body must be visible")

        # only consider people standing inside the target rectangle, everyone else in the frame is ignored
        people_in_rectangle = [person for person in people if is_box_inside(person, target_rectangle)]
        if not people_in_rectangle:
            return self._reject("The person has to be inside the rectangle.")

        # only use a face that actually belongs to the person (avoids pairing person A with face B)
        person, face = select_person(people_in_rectangle, faces)
        if person is None:
            return self._reject("The face of the person inside the rectangle is not visible.")
        if not is_box_inside(face, target_rectangle):
            return self._reject("The face has to be inside the rectangle.")

        area = box_area(person)
        if area < MIN_PERSON_AREA:
            return self._reject(
                f"The person is too far away. (Current area: {area:.0f}, must be over {MIN_PERSON_AREA:.0f})"
            )

        return person

    def _reject(self, reason: str) -> None:
        self._logger.warn(f"Cannot take photo! {reason}")
        return None

    # save only the selected person (cropped from the clean frame, without boxes)
    def _save_photo(self, frame, person, target_rectangle) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(SAVE_DIR, f"person_{stamp}.jpg")
        if not cv2.imwrite(path, crop_to_person(frame, person, target_rectangle)):
            self._logger.error(f"Failed to write {path}")
            self._publish_result(FAILED_RESULT)
            return
        self._logger.info(f"Saved {path}")
        self._publish_result(path)


def main():
    node_runner.run_node(lambda: VisionNode())


if __name__ == "__main__":
    main()
