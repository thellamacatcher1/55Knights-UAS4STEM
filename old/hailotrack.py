import cv2
import argparse
from picamera2 import MappedArray, Picamera2, Preview
from picamera2.devices import Hailo, hailo_architecture

def extract_detections(hailo_output, w, h, class_names, threshold=0.5):
    results = []
    for class_id, detections in enumerate(hailo_output):
        for detection in detections:
            score = detection[4]
            if score >= threshold:
                y0, x0, y1, x1 = detection[:4]
                bbox = (int(x0 * w), int(y0 * h), int(x1 * w), int(y1 * h))
                results.append([class_names[class_id], bbox, score])
    return results

class ObjectDetector:
    def __init__(self, model=None, labels="coco.txt", score_thresh=0.5):
        if model is None:
            model = (
                '/usr/share/hailo-models/yolov8m_h10.hef'
                if hailo_architecture() == 'HAILO10H'
                else '/usr/share/hailo-models/yolov8s_h8l.hef'
            )
        self.model = model
        self.score_thresh = score_thresh
        self.video_w, self.video_h = 1280, 960

        with open(labels, 'r', encoding='utf-8') as f:
            self.class_names = f.read().splitlines()

        self._detections = []
        self._hailo = None
        self._picam2 = None

    def get_detections(self):
        return list(self._detections)

    def _draw_objects(self, request):
        current_detections = self._detections
        with MappedArray(request, "main") as m:
            h, w = m.array.shape[:2]
            cx, cy = w // 2, h // 2

            if current_detections:
                for class_name, bbox, score in current_detections:
                    x0, y0, x1, y1 = bbox
                    label = f"{class_name} %{int(score * 100)}"
                    cv2.rectangle(m.array, (x0, y0), (x1, y1), (0, 255, 0, 0), 2)
                    cv2.putText(m.array, label, (x0 + 5, y0 + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 0), 1,
                                cv2.LINE_AA)

                best = max(current_detections, key=lambda d: d[2])
                x0, y0, x1, y1 = best[1]
                bx, by = (x0 + x1) // 2, (y0 + y1) // 2

                cv2.line(m.array, (cx, cy), (bx, by), (0, 0, 255, 0), 2)
                cv2.circle(m.array, (bx, by), 6, (0, 0, 255, 0), -1)

                offset_x, offset_y = bx - cx, by - cy
                cv2.putText(m.array, f"dx={offset_x:+d} dy={offset_y:+d}",
                            (bx + 10, by - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 255, 0), 1, cv2.LINE_AA)

            cv2.circle(m.array, (cx, cy), 6, (255, 255, 255, 0), -1)
            cv2.drawMarker(m.array, (cx, cy), (200, 200, 200, 0),
                           cv2.MARKER_CROSS, 30, 1)

    def start(self):
        self._hailo = Hailo(self.model)
        self._hailo.__enter__()
        model_h, model_w, _ = self._hailo.get_input_shape()

        self._picam2 = Picamera2()
        main = {'size': (self.video_w, self.video_h), 'format': 'XRGB8888'}
        lores = {'size': (model_w, model_h), 'format': 'RGB888'}
        config = self._picam2.create_preview_configuration(
            main, lores=lores, controls={'FrameRate': 30}
        )
        self._picam2.configure(config)
        self._picam2.start_preview(Preview.QTGL, x=0, y=0,
                                   width=self.video_w, height=self.video_h)
        self._picam2.start()
        self._picam2.pre_callback = self._draw_objects

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
                self._detections = extract_detections(
                    results, self.video_w, self.video_h,
                    self.class_names, self.score_thresh
                )
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Detection Example")
    parser.add_argument("-m", "--model", default=None)
    parser.add_argument("-l", "--labels", default="coco.txt")
    parser.add_argument("-s", "--score_thresh", type=float, default=0.5)
    args = parser.parse_args()

    detector = ObjectDetector(args.model, args.labels, args.score_thresh)
    detector.start()
    detector.run_loop()