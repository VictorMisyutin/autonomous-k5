"""Step 7: object detection with Picamera2 + MediaPipe (COCO labels, e.g. 'stop sign', 'person').

Model download:
  wget -O efficientdet_lite0.tflite \
    https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/int8/1/efficientdet_lite0.tflite
"""
import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from picamera2 import Picamera2

MODEL_PATH = "efficientdet_lite0.tflite"


class Detector:
    def __init__(self, score_threshold=0.5):
        self.cam = Picamera2()
        # Small frames = faster inference on the Pi
        self.cam.configure(self.cam.create_preview_configuration(
            main={"format": "RGB888", "size": (320, 240)}))
        self.cam.start()
        time.sleep(1)  # let the camera warm up
        options = vision.ObjectDetectorOptions(
            base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
            score_threshold=score_threshold,
            max_results=5)
        self.detector = vision.ObjectDetector.create_from_options(options)

    def detect(self):
        """Grab one frame and return the set of detected label names."""
        frame = self.cam.capture_array()             # Picamera2 "RGB888" is BGR order
        # frame = cv2.flip(frame, -1)                # uncomment if your camera is upside down
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.detector.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
        return {d.categories[0].category_name for d in result.detections}

    def close(self):
        self.cam.stop()


if __name__ == "__main__":
    # Quick test: hold a stop sign picture in front of the camera
    det = Detector()
    try:
        while True:
            t = time.time()
            labels = det.detect()
            print(f"{1 / (time.time() - t):.1f} fps  {labels}")
    except KeyboardInterrupt:
        det.close()
