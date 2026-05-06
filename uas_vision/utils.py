import cv2

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


def draw_detections(array, detections, video_w, video_h):
    cx, cy = video_w // 2, video_h // 2
    offset_x, offset_y = 0, 0

    if detections:
        for class_name, bbox, score in detections:
            x0, y0, x1, y1 = bbox
            label = f"{class_name} %{int(score * 100)}"
            cv2.rectangle(array, (x0, y0), (x1, y1), (0, 255, 0, 0), 2)
            cv2.putText(array, label, (x0 + 5, y0 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 0), 1,
                        cv2.LINE_AA)

        best = max(detections, key=lambda d: d[2])
        x0, y0, x1, y1 = best[1]
        bx, by = (x0 + x1) // 2, (y0 + y1) // 2

        offset_x, offset_y = bx - cx, by - cy

        cv2.line(array, (cx, cy), (bx, by), (0, 0, 255, 0), 2)
        cv2.circle(array, (bx, by), 6, (0, 0, 255, 0), -1)
        cv2.putText(array, f"dx={offset_x:+d} dy={offset_y:+d}",
                    (bx + 10, by - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 255, 0), 1, cv2.LINE_AA)
    cv2.circle(array, (cx, cy), 6, (255, 255, 255, 0), -1)
    cv2.drawMarker(array, (cx, cy), (200, 200, 200, 0),
                   cv2.MARKER_CROSS, 30, 1)
    return (offset_x, offset_y)