from picamera2 import MappedArray, Picamera2, Preview
from picamera2.devices import Hailo, hailo_architecture

from uas_vision.utils import extract_detections, draw_detections
import config
import threading
import cv2
import numpy as np

class ObjectDetector:
    def __init__(
        self,
        model=None,
        labels=config.LABELS_PATH,
        score_thresh=config.SCORE_THRESH,
        video_w=config.VIDEO_W,
        video_h=config.VIDEO_H,
        frame_rate=config.FRAME_RATE,
    ):
        if model is None:
            model = (
                '/usr/share/hailo-models/yolov8m_h10.hef'
                if hailo_architecture() == 'HAILO10H'
                else '/usr/share/hailo-models/yolov8s_h8l.hef'
            )

        self.model = model
        self.score_thresh = score_thresh
        self.video_w = video_w
        self.video_h = video_h
        self.frame_rate = frame_rate
        self._lock = threading.Lock()


        with open(labels, 'r', encoding='utf-8') as f:
            self.class_names = f.read().splitlines()

        self._detections = []
        self._offset = (0, 0)
        self._hailo = None
        self._picam2 = None

    def get_detections(self):
        with self._lock:
            return list(self._detections)

    def get_offset(self):
        with self._lock:
            return self._offset

    def start(self):
        self._hailo = Hailo(self.model)
        self._hailo.__enter__()
        model_h, model_w, _ = self._hailo.get_input_shape()

        self._picam2 = Picamera2()
        main = {'size': (self.video_w, self.video_h), 'format': 'XRGB8888'}
        lores = {'size': (model_w, model_h), 'format': 'RGB888'}
        config_cam = self._picam2.create_preview_configuration(
            main, lores=lores, controls={'FrameRate': self.frame_rate}
        )
        self._picam2.configure(config_cam)
        self._picam2.start_preview(Preview.QTGL, x=0, y=0,
                                   width=self.video_w, height=self.video_h)
        self._picam2.start()
        #cropping
        #self._picam2.set_controls({"ScalerCrop": (184, 0, 1088, 1088)})  # ← add here
        self._picam2.pre_callback = self._draw_callback

    def stop(self):
        if self._picam2:
            self._picam2.stop()
            self._picam2.stop_preview()
            self._picam2 = None
        if self._hailo:
            self._hailo.__exit__(None, None, None)
            self._hailo = None

    def run_loop(self):
        try:
            while True:
                frame = self._picam2.capture_array('lores')
                results = self._hailo.run(frame)
                with self._lock:
                    self._detections = extract_detections(
                        results, self.video_w, self.video_h,
                        self.class_names, self.score_thresh
                    )
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            
    def _draw_callback(self, request):
        with MappedArray(request, "main") as m:
            with self._lock:
                self._offset = draw_detections(
                    m.array, self._detections, self.video_w, self.video_h
                )