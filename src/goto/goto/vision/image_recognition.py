import numpy as np
from ultralytics import YOLO


class ImageRecognition:
    """Thin wrapper around a YOLO model that only detects class 0 (person / face, depending on the model)."""

    def __init__(self, recognition_model, conf=0.5):
        self.model = YOLO(recognition_model)
        self.conf = conf
        self.results = []
        self._raw = None  # ultralytics Results of the last detection, used for drawing

    # run detection on a clean BGR frame; returns [(x1, y1, x2, y2, conf), ...]
    def get_results(self, frame: np.ndarray):
        self._raw = self.model(frame, classes=[0], conf=self.conf, verbose=False)[0]
        self.results = [(*box.xyxy[0].tolist(), float(box.conf[0])) for box in self._raw.boxes]
        return self.results

    # draw the last detections onto `canvas` (does not run the model again) and return it
    def draw(self, canvas: np.ndarray) -> np.ndarray:
        if self._raw is None:
            return canvas
        return self._raw.plot(img=canvas)
