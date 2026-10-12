import os
import time
from datetime import datetime

import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String


class VisionNode(NodeWrapper):
    def __init__(self):
        super().__init__("vision", self._setup_channels)

    def _setup_channels(self):
        self._bridge = CvBridge()
        # TODO: change to face recognition model instead of person
        self._model = YOLO("yolov8n.pt")  # small and fast; downloads on first run
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

        # Optional: publish annotated images to view in rviz / rqt_image_view
        self._debug_pub = self.node.create_publisher(Image, "/vision/people_debug", 1)

    #
    def _on_trigger(self, _msg: Empty) -> None:
        if self._state != "idle":
            self.node.get_logger().warn("Capture already in progress, ignoring trigger")
            return
        self._state = "countdown"
        self._countdown_end = time.monotonic() + self._countdown_s
        self._last_logged_second = None
        self.node.get_logger().info(f"Taking picture in {self._countdown_s:.0f} s")

    def _image_callback(self, image: Image) -> None:
        # Drop frames while inference is still running
        if self._busy:
            return
        self._busy = True
        try:
            frame = self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

            results = self._model(frame, classes=[0], conf=0.5, verbose=False)[0]

            people = []
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                people.append((x1, y1, x2, y2, float(box.conf[0])))

            if people:
                pass
                # self.node.get_logger().info(f"Detected {len(people)} person(s)")

            annotated = results.plot()

            # --- Capture state machine ---
            now = time.monotonic()

            if self._state == "countdown":
                remaining = self._countdown_end - now
                if remaining > 0:
                    second = int(remaining) + 1
                    if second != self._last_logged_second:
                        self._last_logged_second = second
                        self.node.get_logger().info(f"{second}...")
                else:
                    self._state = "waiting_for_person"
                    self._wait_end = now + self._person_timeout_s

            if self._state == "waiting_for_person":
                if people:
                    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    path = os.path.join(self._save_dir, f"person_{stamp}.jpg")
                    cv2.imwrite(path, frame)  # use `annotated` to include the boxes
                    self.node.get_logger().info(f"Saved {path}")
                    self._result_pub.publish(String(data=path))
                    self._state = "idle"
                elif now > self._wait_end:
                    self.node.get_logger().warn("No person in frame, capture cancelled")
                    self._result_pub.publish(String(data=""))  # empty = failed
                    self._state = "idle"

            # Publish image with boxes drawn
            out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = image.header
            self._debug_pub.publish(out)
        finally:
            self._busy = False


def main():
    node_runner.run_node(lambda: VisionNode())


if __name__ == '__main__':
    main()
