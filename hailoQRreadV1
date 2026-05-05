#!/usr/bin/env python3
"""
Hailo Detection + Dynamsoft QR Scanner — merged
================================================
Hailo  : draws bounding box every frame at 30fps (always smooth)
Dynamsoft : runs in background thread every N frames, decodes QR text
Result : box tracks at full speed from Hailo, decoded text persists on top
"""

import argparse
import threading
import time

import cv2
import numpy as np
from picamera2 import MappedArray, Picamera2, Preview
from picamera2.devices import Hailo, hailo_architecture
from dynamsoft_barcode_reader_bundle import (
    LicenseManager, CaptureVisionRouter,
    EnumPresetTemplate, EnumImagePixelFormat,
    EnumCapturedResultItemType, ImageData,
)

# ── Config ────────────────────────────────────────────────────────────────────

API_KEY        = "t0087YQEAAIEGbdhak1Zx9dhH6g290e4lAzFzddzxsjerD48x5mrGbY3M8fbQ3ifjldRz+EDogg1c+Txoeku3jvabfdUqEtTLD6Z9NLmbhfemB+KtxQbV0kmW"
DECODE_EVERY_N = 5           # decode QR text every Nth frame

# ── Shared state ──────────────────────────────────────────────────────────────

# Hailo detection state (written by main loop, read by draw_objects callback)
detections      = None

# Dynamsoft decode state (written by decode thread, read by draw_objects)
_decode_lock    = threading.Lock()
_decoded_text   = ""         # last successfully decoded QR text — persists
_decoder_busy   = False


# ── Dynamsoft decode worker ───────────────────────────────────────────────────

def _decode_worker(cvr, frame_bytes, w, h):
    global _decoded_text, _decoder_busy

    image_data = ImageData(
        frame_bytes, w, h, w * 3,
        EnumImagePixelFormat.IPF_RGB_888,
    )
    result = cvr.capture(image_data, EnumPresetTemplate.PT_READ_BARCODES)

    text = ""
    if result:
        for item in result.get_items():
            if item.get_type() == EnumCapturedResultItemType.CRIT_BARCODE:
                text = item.get_text()
                break   # take first result

    with _decode_lock:
        if text:                 # only update on success — never wipe on miss
            _decoded_text = text
        _decoder_busy = False


# ── Hailo helpers (unchanged from original) ───────────────────────────────────

def extract_detections(hailo_output, w, h, class_names, threshold=0.5):
    results = []
    for class_id, dets in enumerate(hailo_output):
        for detection in dets:
            score = detection[4]
            if score >= threshold:
                y0, x0, y1, x1 = detection[:4]
                bbox = (int(x0 * w), int(y0 * h), int(x1 * w), int(y1 * h))
                results.append([class_names[class_id], bbox, score])
    return results


# ── Draw callback (runs on camera thread every frame) ─────────────────────────

def draw_objects(request):
    current_detections = detections

    with _decode_lock:
        qr_text = _decoded_text      # grab latest decoded text (may be "")

    with MappedArray(request, "main") as m:
        h, w = m.array.shape[:2]
        cx = w // 2
        cy = h // 2

        if current_detections:
            # Draw all bounding boxes — unchanged from original
            for class_name, bbox, score in current_detections:
                x0, y0, x1, y1 = bbox
                label = f"{class_name} %{int(score * 100)}"
                cv2.rectangle(m.array, (x0, y0), (x1, y1), (0, 255, 0, 0), 2)
                cv2.putText(m.array, label, (x0 + 5, y0 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0, 0), 1,
                            cv2.LINE_AA)

            # Best detection — unchanged from original
            best = max(current_detections, key=lambda d: d[2])
            x0, y0, x1, y1 = best[1]
            bx = (x0 + x1) // 2
            by = (y0 + y1) // 2

            cv2.line(m.array, (cx, cy), (bx, by), (0, 0, 255, 0), 2)
            cv2.circle(m.array, (bx, by), 6, (0, 0, 255, 0), -1)

            offset_x = bx - cx
            offset_y = by - cy
            offset_label = f"dx={offset_x:+d} dy={offset_y:+d}"
            cv2.putText(m.array, offset_label, (bx + 10, by - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255, 0), 1,
                        cv2.LINE_AA)

            # ── NEW: decoded QR text drawn on top of best bbox ────────────────
            if qr_text:
                display = qr_text[:28] + "..." if len(qr_text) > 28 else qr_text
                cv2.putText(m.array, display,
                            (x0 + 5, y1 - 8),           # bottom-left of bbox
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255, 0), 2, cv2.LINE_AA)

        # Centre dot + crosshair — unchanged from original
        cv2.circle(m.array, (cx, cy), 6, (255, 255, 255, 0), -1)
        cv2.drawMarker(m.array, (cx, cy), (200, 200, 200, 0),
                       cv2.MARKER_CROSS, 30, 1)


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    global detections, _decoder_busy

    # ── Dynamsoft init ────────────────────────────────────────────────────────
    err, msg = LicenseManager.init_license(API_KEY)
    print(f"[LICENSE] {err} — {msg}")
    if err != 0:
        print("License failed, exiting.")
        raise SystemExit(1)
    cvr = CaptureVisionRouter()

    # ── Args — unchanged from original ───────────────────────────────────────
    parser = argparse.ArgumentParser(description="Hailo + QR")
    parser.add_argument("-m", "--model", default=None)
    parser.add_argument("-l", "--labels", default="coco.txt")
    parser.add_argument("-s", "--score_thresh", type=float, default=0.5)
    args = parser.parse_args()

    if args.model is None:
        args.model = ('/usr/share/hailo-models/yolov8m_h10.hef'
                      if hailo_architecture() == 'HAILO10H'
                      else '/usr/share/hailo-models/yolov8s_h8l.hef')

    with Hailo(args.model) as hailo:
        model_h, model_w, _ = hailo.get_input_shape()
        video_w, video_h = 1280, 960

        with open(args.labels, 'r', encoding="utf-8") as f:
            class_names = f.read().splitlines()

        detections = None

        with Picamera2() as picam2:
            main   = {'size': (video_w, video_h), 'format': 'XRGB8888'}
            lores  = {'size': (model_w, model_h), 'format': 'RGB888'}
            controls = {'FrameRate': 30}
            config = picam2.create_preview_configuration(main, lores=lores,
                                                         controls=controls)
            picam2.configure(config)
            picam2.start_preview(Preview.QTGL, x=0, y=0,
                                 width=video_w, height=video_h)
            picam2.start()
            picam2.pre_callback = draw_objects

            frame_count = 0

            while True:
                # ── Hailo inference — unchanged from original ─────────────────
                frame = picam2.capture_array('lores')
                results = hailo.run(frame)
                detections = extract_detections(results, video_w, video_h,
                                                class_names, args.score_thresh)

                # ── Dynamsoft — fire every N frames, non-blocking ─────────────
                frame_count += 1
                with _decode_lock:
                    busy = _decoder_busy

                if not busy and frame_count % DECODE_EVERY_N == 0:
                    # Grab a fresh full-res frame for the decoder
                    # lores is too small — use the main stream
                    decode_raw = picam2.capture_array('main')

                    # Convert XRGB → RGB for Dynamsoft
                    if decode_raw.shape[2] == 4:
                        decode_rgb = cv2.cvtColor(decode_raw, cv2.COLOR_RGBA2RGB)
                    else:
                        decode_rgb = cv2.cvtColor(decode_raw, cv2.COLOR_BGR2RGB)

                    dh, dw = decode_rgb.shape[:2]

                    with _decode_lock:
                        _decoder_busy = True

                    threading.Thread(
                        target=_decode_worker,
                        args=(cvr, decode_rgb.tobytes(), dw, dh),
                        daemon=True,
                    ).start()
