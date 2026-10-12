from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO


class ImageRecognition:
    def __init__(self, node, recognition_model):
        self.bridge = CvBridge()
        self.model = YOLO(
            recognition_model
        )
        self.results = []
        self.frame = None
        self.results = []
        self.annotated = None
        # Optional: publish annotated images to view in rviz / rqt_image_view
        self._debug_pub = node.create_publisher(Image, "/vision/people_debug", 1)

    def get_results(self, image):
        # clear previous
        self.results = []
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        self.frame = frame
        results = self.model(frame, classes=[0], conf=0.5, verbose=False)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            self.results.append((x1, y1, x2, y2, float(box.conf[0])))
        self.annotated = results.plot()
        return self.results

    def publish_results(self, image):
        out = self.bridge.cv2_to_imgmsg(self.annotated, encoding="bgr8")
        out.header = image.header
        self._debug_pub.publish(out)
