import threading
import time
import cv2
import numpy as np
from picamera2 import Picamera2, Preview, MappedArray
from picamera2.devices import Hailo, hailo_architecture

import config

class ObjectDetector:
    def __init__(self, model, labels, score_thresh=0.5):
        self.model = model
        self.labels = labels
        self.score_thresh = score_thresh

        self.detections = []
        self.offset_x = 0
        self.offset_y = 0
        self._detections_lock = threading.Lock()

        self._detector_thread = None
        self._stop_event = threading.Event()
        self.is_running = False

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": (config.VIDEO_W, config.VIDEO_H)},
            lores={"size": (640, 480)},
            display="lores"
        ))
        self.picam2.start()

        self.hailo = Hailo()
        self.hailo.set_model(self.model)
        self.hailo.set_labels(self.labels)
        self.hailo.set_thresholds(self.score_thresh)

        self.labels_list = self.hailo.get_labels()

    def start(self):
        if not self.is_running:
            self.is_running = True
            self._stop_event.clear()
            self._detector_thread = threading.Thread(target=self._detection_loop, daemon=True)
            self._detector_thread.start()
            print("Detection started")

    def stop(self):
        if self.is_running:
            self._stop_event.set()
            self.is_running = False
            if self._detector_thread:
                self._detector_thread.join()
            print("Detection stopped")

    def _detection_loop(self):
        try:
            while not self._stop_event.is_set():
                frame = self.picam2.capture_array()
                detections = self.hailo.run_detection(frame)

                with self._detections_lock:
                    self.detections = detections

                    if detections:
                        best_detection = max(detections, key=lambda d: d[2])
                        class_name, bbox, score = best_detection
                        x0, y0, x1, y1 = bbox
                        center_x = (x0 + x1) // 2
                        center_y = (y0 + y1) // 2

                        frame_center_x = config.VIDEO_W // 2
                        frame_center_y = config.VIDEO_H // 2

                        self.offset_x = center_x - frame_center_x
                        self.offset_y = center_y - frame_center_y
                    else:
                        self.offset_x = 0
                        self.offset_y = 0

                time.sleep(0.05)

        except Exception as e:
            print(f"Error in detection loop: {e}")

    def get_detections(self):
        with self._detections_lock:
            return self.detections.copy()

    def get_offset(self):
        with self._detections_lock:
            return self.offset_x, self.offset_y

    def run_loop(self):
        self._detection_loop()