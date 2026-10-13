import os
import time
from datetime import datetime

import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String

from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper
from go2_nav.vision.image_recognition import ImageRecognition


# TODO: only take picture when person is in a specific box
# TODO: change timer logic

class VisionNode(NodeWrapper):
    def __init__(self):
        super().__init__("vision", self._setup_channels)

    def print_log(self, log, msg_type):
        if msg_type == "warn":
            self.node.get_logger().warn(log)
        elif msg_type == "info":
            self.node.get_logger().info(log)
        elif msg_type == "error":
            self.node.get_logger().error(log)


    def _setup_channels(self):
        self._person_recognition = ImageRecognition(
            self.node,
            "yolov8n.pt"
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
        if self._state != "idle":
            self.print_log("Capture already in progress, ignoring trigger", "warn")
            return
        self._state = "countdown"
        self._countdown_end = time.monotonic() + self._countdown_s
        self._last_logged_second = None
        self.print_log(f"Taking picture in {self._countdown_s:.0f} s", "info")

    def _image_callback(self, image: Image) -> None:
        # Drop frames while inference is still running
        if self._busy:
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

            frame = self._person_recognition.annotated
            height, width = frame.shape[:2]

            box_width = 150  # adjust these to set the box size
            box_height = 400
            x1 = (width - box_width) // 2
            y1 = (height - box_height) // 2
            x2 = x1 + box_width
            y2 = y1 + box_height

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)


            # Publish image with boxes drawn
            # self._face_recognition.publish_results(image)
            self._person_recognition.publish_results(image)

            # --- Capture state machine ---
            now = time.monotonic()

            if self._state == "countdown":
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
                if faces and people:
                    self.save_image()
                    self._state = "idle"
                elif now > self._wait_end:
                    self.print_log("No person in frame, capture cancelled", "warn")
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
