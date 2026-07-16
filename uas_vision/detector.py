import threading
import time
from picamera2 import Picamera2, Preview, MappedArray
from picamera2.devices import Hailo
import config
import cv2
from uas_vision.utils import extract_detections, draw_detections


class ObjectDetector:
    def __init__(self, model, labels, score_thresh=0.5, headless=False):
        self.model = model
        self.score_thresh = score_thresh
        self.headless = headless
        self.detections = []
        self.offset_x = 0
        self.offset_y = 0
        self.offsets_by_class = {}
        self._detections_lock = threading.Lock()
        self._detector_thread = None
        self._stop_event = threading.Event()
        self.is_running = False

        with open(labels, 'r', encoding='utf-8') as f:
            self.class_names = f.read().splitlines()

        self.hailo = Hailo(self.model)
        model_h, model_w, _ = self.hailo.get_input_shape()

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": (config.VIDEO_W, config.VIDEO_H)},
            lores={"size": (model_w, model_h), "format": "RGB888"},
        ))

        if not self.headless:
            self.picam2.start_preview(Preview.QTGL, x=0, y=0,
                                       width=config.VIDEO_W, height=config.VIDEO_H)
        self.picam2.start()

        if not self.headless:
            self.picam2.pre_callback = self._draw_callback

    def _draw_callback(self, request):
        with MappedArray(request, "main") as m:
            with self._detections_lock:
                draw_detections(m.array, self.detections, config.VIDEO_W, config.VIDEO_H)

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
            if not self.headless:
                self.picam2.stop_preview()
            self.picam2.stop()
            print("Detection stopped")

    def _detection_loop(self):
        try:
            while not self._stop_event.is_set():
                frame = self.picam2.capture_array('lores')
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # test: try the opposite channel order

                hailo_output = self.hailo.run(frame)
                detections = extract_detections(
                    hailo_output, config.VIDEO_W, config.VIDEO_H,
                    self.class_names, self.score_thresh
                )

                with self._detections_lock:
                    self.detections = detections

                    offsets_by_class = {}
                    for class_name, bbox, score in detections:
                        x0, y0, x1, y1 = bbox
                        center_x = (x0 + x1) // 2
                        center_y = (y0 + y1) // 2
                        dx = center_x - (config.VIDEO_W // 2)
                        dy = center_y - (config.VIDEO_H // 2)
                        if class_name not in offsets_by_class or score > offsets_by_class[class_name][2]:
                            offsets_by_class[class_name] = (dx, dy, score)
                    self.offsets_by_class = {
                        c: (dx, dy) for c, (dx, dy, s) in offsets_by_class.items()
                    }

                    if detections:
                        best_detection = max(detections, key=lambda d: d[2])
                        class_name, bbox, score = best_detection
                        x0, y0, x1, y1 = bbox
                        center_x = (x0 + x1) // 2
                        center_y = (y0 + y1) // 2
                        self.offset_x = center_x - (config.VIDEO_W // 2)
                        self.offset_y = center_y - (config.VIDEO_H // 2)
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
            return