import os
import time
from datetime import datetime

import cv2
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


# TODO: change timer logic

# TODO: only take photo of the closest person

class VisionNode(NodeWrapper):
    def __init__(self):
        super().__init__("vision", self._setup_channels)

        self.active_states = frozenset({State.WAIT_FOR_PERSON})


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
        return (
            box_x1 >= rect_x1
            and box_y1 >= rect_y1
            and box_x2 <= rect_x2
            and box_y2 <= rect_y2
        )

    # calculate area of a box
    @staticmethod
    def _box_area(box) -> float:
        x1, y1, x2, y2 = box[:4]
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    # check if person is perfectly visible
    def _is_person_correctly_visible(self, people, faces, target_rectangle) -> bool:
        if not people and not faces:
            self.print_log("Cannot take photo! No person detected.", "warn")
            return False
        if people and not faces:
            self.print_log("Cannot take photo! Either too close or the face is obstructed", "warn")
            return False
        if not people and faces:
            self.print_log("Cannot take photo! The person must face the camera and the entire body must be visible",
                           "warn")
            return False

        # check if person and face is inside the target rectangle (green rectangle)

        # TODO: select correct person
        is_person_in_rectangle = self._is_box_inside_rectangle(people[0], target_rectangle)
        is_face_in_rectangle = self._is_box_inside_rectangle(faces[0], target_rectangle)

        if not is_person_in_rectangle or not is_face_in_rectangle:
            self.print_log("Cannot take photo! The person has to be inside the rectangle.", "warn")
            return False

        # TODO: select correct person
        area_person = self._box_area(people[0])
        if area_person < MIN_PERSON_AREA:
            self.print_log(
                f"Cannot take photo! The person is too far away. (Current area: {area_person:.0f}, must be over {MIN_PERSON_AREA:.0f})",
                "warn")
            return False

        return True

    def _setup_channels(self):
        person_model_path = os.path.join("weights", "yolov8n.pt")
        os.makedirs(os.path.dirname(person_model_path), exist_ok=True)
        self._person_recognition = ImageRecognition(
            self.node,
            person_model_path
        )

        self._face_recognition = ImageRecognition(
            self.node,
            "https://github.com/akanametov/yolo-face/releases/download/1.0.0/yolov8n-face.pt"
        )

        self._busy = False

        # Capture state: "idle" -> "countdown" -> "waiting_for_person" -> "idle"
        self._save_dir = os.path.expanduser("~/go2_detections")
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
        self._trigger_sub = self.node.create_subscription(
            Empty, "/vision/take_picture", self._on_trigger, 10
        )
        self._result_pub = self.node.create_publisher(String, "/vision/picture_result", 10)

    # this starts the photo mode
    def _on_trigger(self, _msg: Empty) -> None:
        if not self.is_active():
            self.print_log("Cannot capture, state is not active", "warn")
            return
        if self._state != "idle":
            self.print_log("Capture already in progress, ignoring trigger", "warn")
            return
        self._state = "countdown"
        self._countdown_end = time.monotonic() + self._countdown_s
        self._last_logged_second = None
        self.print_log(f"Trying to take a picture in {self._countdown_s:.0f} s", "info")

    def _image_callback(self, image: Image) -> None:
        # only activate when the current global state is WAIT_FOR_PERSON
        if self._busy or not self.is_active():
            return
        self._busy = True
        try:

            # get people who are in frame and draw boxes around them
            faces = self._face_recognition.get_results(image)

            # merge boxes from faces and people together
            people = self._person_recognition.get_results(
                self._person_recognition.bridge.cv2_to_imgmsg(
                    self._face_recognition.annotated, encoding="bgr8"
                )
            )

            # draw box where person is supposed to stand
            frame = self._person_recognition.annotated
            height, width = frame.shape[:2]

            x1 = (width - BOX_WIDTH) // 2
            y1 = (height - BOX_HEIGHT) // 2
            x2 = x1 + BOX_WIDTH
            y2 = y1 + BOX_HEIGHT

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            target_rectangle = (x1, y1, x2, y2)

            """
            if people:
                area = self._box_area(people[0])
                if area > 30000:
                    self.print_log(f"Person area: {area:.0f} px²", "info")
            """

            # publish image with boxes drawn
            # self._face_recognition.publish_results(image)
            self._person_recognition.publish_results(image)

            # --- Capture state machine ---
            now = time.monotonic()

            if self._state == "countdown":
                # return to idle state if the person is not standing in front of the camera properly
                if not self._is_person_correctly_visible(people, faces, target_rectangle):
                    self.print_log("Countdown cancelled!", "warn")
                    self._state = "idle"
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
                if self._is_person_correctly_visible(people, faces, target_rectangle):
                    # TODO: only crop person
                    self.save_image()
                    self._state = "idle"
                elif now > self._wait_end:
                    self.print_log(
                        "No person and face inside the target box, capture cancelled",
                        "warn",
                    )
                    self._result_pub.publish(String(data=""))  # empty = failed
                    self._state = "idle"

        finally:
            self._busy = False

    def save_image(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(self._save_dir, f"person_{stamp}.jpg")
        cv2.imwrite(path, self._face_recognition.frame)  # use `annotated` to include the boxes
        self.print_log(f"Saved {path}", "info")
        self._result_pub.publish(String(data=path))


def main():
    node_runner.run_node(lambda: VisionNode())


if __name__ == '__main__':
    main()
