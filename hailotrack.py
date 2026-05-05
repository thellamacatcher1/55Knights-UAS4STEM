import argparse
import cv2
from picamera2 import MappedArray, Picamera2, Preview
from picamera2.devices import Hailo, hailo_architecture


def extract_detections(hailo_output, w, h, class_names, threshold=0.5):
    # Extract detections from the HailoRT-postprocess output.
    results = []
    for class_id, detections in enumerate(hailo_output):
        for detection in detections:
            score = detection[4]
            if score >= threshold:
                y0, x0, y1, x1 = detection[:4]
                bbox = (int(x0 * w), int(y0 * h), int(x1 * w), int(y1 * h))
                results.append([class_names[class_id], bbox, score])
    return results


def draw_objects(request):
    current_detections = detections
    with MappedArray(request, "main") as m:
        h, w = m.array.shape[:2]
        cx = w // 2
        cy = h // 2

        # Draw all  boxes first
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
            bx = (x0 + x1) // 2
            by = (y0 + y1) // 2

            cv2.line(m.array, (cx, cy), (bx, by), (0, 0, 255, 0), 2)

            cv2.circle(m.array, (bx, by), 6, (0, 0, 255, 0), -1)

            offset_x = bx - cx
            offset_y = by - cy   # positive = below centre (image coords)
            offset_label = f"dx={offset_x:+d} dy={offset_y:+d}"
            cv2.putText(m.array, offset_label, (bx + 10, by - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255, 0), 1,
                        cv2.LINE_AA)

        # Dot at frame centr (white) — always drawn
        cv2.circle(m.array, (cx, cy), 6, (255, 255, 255, 0), -1)
        cv2.drawMarker(m.array, (cx, cy), (200, 200, 200, 0),
                       cv2.MARKER_CROSS, 30, 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Detection Example")
    parser.add_argument("-m", "--model",
                        help="Path for the HEF model. "
                             "Defaults to /usr/share/hailo-models/yolov8s_h8l.hef for H8 devices, "
                             "and /usr/share/hailo-models/yolov8m_h10.hef for H10 devices.",
                        default=None)
    parser.add_argument("-l", "--labels", default="coco.txt",
                        help="Path to a text file containing labels.")
    parser.add_argument("-s", "--score_thresh", type=float, default=0.5,
                        help="Score threshold, must be a float between 0 and 1.")
    args = parser.parse_args()

    if args.model is None:
        if hailo_architecture() == 'HAILO10H':
            args.model = '/usr/share/hailo-models/yolov8m_h10.hef'
        else:
            args.model = '/usr/share/hailo-models/yolov8s_h8l.hef'

    with Hailo(args.model) as hailo:
        model_h, model_w, _ = hailo.get_input_shape()
        video_w, video_h = 1280, 960

        with open(args.labels, 'r', encoding="utf-8") as f:
            class_names = f.read().splitlines()

        detections = None

        with Picamera2() as picam2:
            main = {'size': (video_w, video_h), 'format': 'XRGB8888'}
            lores = {'size': (model_w, model_h), 'format': 'RGB888'}
            controls = {'FrameRate': 30}
            config = picam2.create_preview_configuration(main, lores=lores, controls=controls)
            picam2.configure(config)
            picam2.start_preview(Preview.QTGL, x=0, y=0, width=video_w, height=video_h)
            picam2.start()
            picam2.pre_callback = draw_objects

            while True:
                frame = picam2.capture_array('lores')
                results = hailo.run(frame)
                detections = extract_detections(results, video_w, video_h,
                                                class_names, args.score_thresh)