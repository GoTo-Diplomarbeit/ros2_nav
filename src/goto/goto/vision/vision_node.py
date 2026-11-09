import os
import time
from datetime import datetime

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper
from goto.state.mission_state import State
from goto.vision.image_recognition import ImageRecognition

# TODO: check if crop works

# minimum box area of person
# aka how close the person is to the camera
MIN_PERSON_AREA = 28000

# box width and height where person is supposed to stand in
# box is always centered
BOX_WIDTH = 250
BOX_HEIGHT = 465

# a face belongs to a person if the face center lies inside the person box
# and within this upper fraction of the person box (the head is at the top)
FACE_TOP_FRACTION = 0.4

# extra margin around the person box when cropping the saved photo (fraction of box width / height)
CROP_PADDING = 0.1


# TODO: change timer logic


class VisionNode(NodeWrapper):
    def __init__(self):
        super().__init__("vision", self._setup_channels)

        # todo for later: add more states when the other states get implemented
        self.active_states = frozenset({State.REGISTER_PERSON})

    def print_log(self, log, msg_type):
        if msg_type == "warn":
            self.node.get_logger().warn(log)
        elif msg_type == "info":
            self.node.get_logger().info(log)
        elif msg_type == "error":
            self.node.get_logger().error(log)

    # check if box is inside a drawn rectangle
    @staticmethod
    def _is_box_inside_rectangle(box, rectangle) -> bool:
        box_x1, box_y1, box_x2, box_y2 = box[:4]
        rect_x1, rect_y1, rect_x2, rect_y2 = rectangle
        # check if box is inside rectangle
        return box_x1 >= rect_x1 and box_y1 >= rect_y1 and box_x2 <= rect_x2 and box_y2 <= rect_y2

    # calculate area of a box
    @staticmethod
    def _box_area(box) -> float:
        x1, y1, x2, y2 = box[:4]
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    # find the face that belongs to a person box (face center in the upper part of the person box)
    @staticmethod
    def _face_for_person(person, faces):
        px1, py1, px2, py2 = person[:4]
        head_limit = py1 + (py2 - py1) * FACE_TOP_FRACTION
        best = None
        for face in faces:
            cx = (face[0] + face[2]) / 2
            cy = (face[1] + face[3]) / 2
            if px1 <= cx <= px2 and py1 <= cy <= head_limit:
                if best is None or face[4] > best[4]:  # highest confidence wins
                    best = face
        return best

    # pick the closest person (largest box) that has a matching face -> (person, face) or (None, None)
    def _select_person(self, people, faces):
        for person in sorted(people, key=self._box_area, reverse=True):
            face = self._face_for_person(person, faces)
            if face is not None:
                return person, face
        return None, None

    # check if person is perfectly visible; returns the selected person box, or None if no valid person
    def _get_visible_person(self, people, faces, target_rectangle):
        if not people and not faces:
            self.print_log("Cannot take photo! No person detected.", "warn")
            return None
        if people and not faces:
            self.print_log("Cannot take photo! Either too close or the face is obstructed", "warn")
            return None
        if not people and faces:
            self.print_log(
                "Cannot take photo! The person must face the camera and the entire body must be visible", "warn"
            )
            return None

        # only use a face that actually belongs to the person (avoids pairing person A with face B)
        person, face = self._select_person(people, faces)
        if person is None:
            self.print_log("Cannot take photo! The detected face does not belong to a detected person.", "warn")
            return None

        # check if person and face is inside the target rectangle (green rectangle)
        is_person_in_rectangle = self._is_box_inside_rectangle(person, target_rectangle)
        is_face_in_rectangle = self._is_box_inside_rectangle(face, target_rectangle)

        if not is_person_in_rectangle or not is_face_in_rectangle:
            self.print_log("Cannot take photo! The person has to be inside the rectangle.", "warn")
            return None

        area_person = self._box_area(person)
        if area_person < MIN_PERSON_AREA:
            self.print_log(
                "Cannot take photo! The person is too far away."
                + f" (Current area: {area_person:.0f}, must be over {MIN_PERSON_AREA:.0f})",
                "warn",
            )
            return None

        return person

    def _setup_channels(self):
        person_model_path = os.path.join("weights", "yolov8n.pt")
        os.makedirs(os.path.dirname(person_model_path), exist_ok=True)
        self._person_recognition = ImageRecognition(person_model_path)

        self._face_recognition = ImageRecognition(
            "https://github.com/akanametov/yolo-face/releases/download/1.0.0/yolov8n-face.pt"
        )

        self._bridge = CvBridge()
        self._frame = None  # last clean camera frame (used when saving the photo)
        # annotated image (faces + people + target rectangle) to view in rviz / rqt_image_view
        self._debug_pub = self.node.create_publisher(Image, "/vision/person_recognition", 1)

        self._busy = False

        # Capture state: "idle" -> "countdown" -> "waiting_for_person" -> "idle"
        # TODO: temporary hardcoded path, switch back to a configurable / home-relative dir
        self._save_dir = "/home/gazebo/go2_detections"
        os.makedirs(self._save_dir, exist_ok=True)
        self._state = "idle"
        self._countdown_s = 5.0
        self._person_timeout_s = 10.0  # how long to wait for a person after the countdown
        self._countdown_end = 0.0
        self._wait_end = 0.0
        self._last_logged_second = None

        self._image_subscription = self.node.create_subscription(
            Image,
            "/d455/image",
            self._image_callback,
            1,  # small queue so you always process a recent frame
        )

        # Trigger from the other node, and result back to it TODO::: get from the correct node
        self._trigger_sub = self.node.create_subscription(Empty, "/vision/take_picture", self._on_trigger, 10)
        self._result_pub = self.node.create_publisher(String, "/vision/picture_result", 10)

    # this starts the photo mode
    def _on_trigger(self, _msg: Empty) -> None:
        if not self.is_active():
            self.print_log("Cannot capture, state is not active", "warn")
            self._result_pub.publish(String(data=""))  # empty = failed
            return
        if self._state != "idle":
            self.print_log("Capture already in progress, ignoring trigger", "warn")
            return
        self._state = "countdown"
        self._countdown_end = time.monotonic() + self._countdown_s
        self._last_logged_second = None
        self.print_log(f"Trying to take a picture in {self._countdown_s:.0f} s", "info")

    # mission state left REGISTER_PERSON (or went unknown): abort a running capture,
    # otherwise it would resume with expired timers the next time the state is entered
    def on_state_exit(self, state: State) -> None:
        if self._state != "idle":
            self.print_log(f"Mission state left {state.value}, capture cancelled", "warn")
            self._cancel_capture()

    # reset the capture and tell the requester that it failed
    def _cancel_capture(self) -> None:
        self._state = "idle"
        self._result_pub.publish(String(data=""))  # empty = failed

    def _image_callback(self, image: Image) -> None:
        # only activate when the current global state is REGISTER_PERSON
        if self._busy or not self.is_active():
            return
        self._busy = True
        try:
            # run both models on the same clean frame (never on an image with boxes drawn on it)
            frame = self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            self._frame = frame
            faces = self._face_recognition.get_results(frame)
            people = self._person_recognition.get_results(frame)

            # merge: draw face and person boxes onto one copy of the frame
            annotated = self._face_recognition.draw(frame.copy())
            annotated = self._person_recognition.draw(annotated)

            # draw box where person is supposed to stand
            height, width = frame.shape[:2]

            x1 = (width - BOX_WIDTH) // 2
            y1 = (height - BOX_HEIGHT) // 2
            x2 = x1 + BOX_WIDTH
            y2 = y1 + BOX_HEIGHT

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            target_rectangle = (x1, y1, x2, y2)

            """
            if people:
                area = self._box_area(people[0])
                if area > 30000:
                    self.print_log(f"Person area: {area:.0f} px²", "info")
            """

            # publish image with boxes drawn
            out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = image.header
            self._debug_pub.publish(out)

            # --- Capture state machine ---
            now = time.monotonic()

            if self._state == "countdown":
                # return to idle state if the person is not standing in front of the camera properly
                # TODO: a single bad frame (YOLO detection flicker) cancels the whole countdown -
                #  add a grace period (e.g. only cancel after N bad frames in a row / ~0.5 s)
                if self._get_visible_person(people, faces, target_rectangle) is None:
                    self.print_log("Countdown cancelled!", "warn")
                    self._cancel_capture()
                    return

                remaining = self._countdown_end - now
                if remaining > 0:
                    second = int(remaining) + 1
                    if second != self._last_logged_second:
                        self._last_logged_second = second
                        self.print_log(f"{second}...", "info")
                else:
                    self._state = "waiting_for_person"
                    self._wait_end = now + self._person_timeout_s

            # CAPTURE
            if self._state == "waiting_for_person":
                # check if person is actually facing the camera
                person = self._get_visible_person(people, faces, target_rectangle)
                if person is not None:
                    self.save_image(person)
                    self._state = "idle"
                elif now > self._wait_end:
                    self.print_log(
                        "No person and face inside the target box, capture cancelled",
                        "warn",
                    )
                    self._cancel_capture()

        finally:
            self._busy = False

    # crop the frame to the person box (plus CROP_PADDING), clamped to the image borders
    @staticmethod
    def _crop_to_person(frame, person):
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = person[:4]
        pad_x = (x2 - x1) * CROP_PADDING
        pad_y = (y2 - y1) * CROP_PADDING
        x1 = max(0, int(x1 - pad_x))
        y1 = max(0, int(y1 - pad_y))
        x2 = min(width, int(x2 + pad_x))
        y2 = min(height, int(y2 + pad_y))
        return frame[y1:y2, x1:x2]

    # save only the selected person (cropped from the clean frame, without boxes)
    def save_image(self, person):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(self._save_dir, f"person_{stamp}.jpg")
        cv2.imwrite(path, self._crop_to_person(self._frame, person))
        self.print_log(f"Saved {path}", "info")
        self._result_pub.publish(String(data=path))


def main():
    node_runner.run_node(lambda: VisionNode())


if __name__ == "__main__":
    main()
