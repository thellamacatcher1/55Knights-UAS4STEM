#!/usr/bin/env python3
#| Argument        | Description                 | Default                   |
#|----------       |-------------                |---------                  |
#| `--track`       | Enable tracking mode        | False                     |
#| `--target`      | Target class to track       | "hammer"                  |
#| `--lower_speed` | Lowering speed (pixels/sec) | 0.01                      |
#| `--mavlink`     | MAVLink connection string   | config.MAVLINK_CONNECTION |
#| `--baud`        | MAVLink baud rate           | config.MAVLINK_BAUD       |
#| `--dry-run`     | No real MAVLink; print velocity commands only | False   |
import argparse
import threading
import time
import config
from uas_vision import ObjectDetector
from uas_mavlink import Drone
from uas_mavlink.controller import TrackingController


class DryRunDrone:
    """Stands in for Drone during bench tests — no MAVLink connection at all.
    Just prints what would have been sent, so you can watch vx/vy react to
    a target moved by hand under the camera without any Pixhawk attached."""
    def set_velo_body(self, vx, vy, vz):
        print(f"[DryRun] set_velo_body -> vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}")

    def get_location(self):
        return None  # forces altitude-hold branch off, since that's still commented out anyway


def parse_args():
    parser = argparse.ArgumentParser(description="Drone Vision + MAVLink")
    parser.add_argument("-m", "--model",        default=config.MODEL_PATH)
    parser.add_argument("-l", "--labels",       default=config.LABELS_PATH)
    parser.add_argument("-s", "--score_thresh", type=float, default=config.SCORE_THRESH)
    parser.add_argument("--mavlink", default=config.MAVLINK_CONNECTION)
    parser.add_argument("--baud",    type=int, default=config.MAVLINK_BAUD)
    parser.add_argument("--track", action="store_true", help="Enable tracking mode")
    parser.add_argument("--target", default="hammer", help="Target class to track")
    parser.add_argument("--lower_speed", type=float, default=0.01, help="Lowering speed (pixels per second)")
    parser.add_argument("--dry-run", action="store_true",
                         help="Skip MAVLink entirely, use a stub that prints velocity commands")
    return parser.parse_args()


def main():
    args = parse_args()

    detector = ObjectDetector(
        model=args.model,
        labels=args.labels,
        score_thresh=args.score_thresh,
    )
    detector.start()

    drone = None
    if args.dry_run:
        drone = DryRunDrone()
        print("[Main] Dry-run mode: no MAVLink connection, printing commands only")
    elif args.mavlink:
        try:
            drone = Drone(args.mavlink, args.baud)
            drone.connect()
            print("Drone connected")
        except Exception as e:
            print(f"Drone connection failed: {e}")

    vision_thread = threading.Thread(target=detector.run_loop, daemon=True)
    vision_thread.start()

    if args.track:
        print(f"Tracking mode enabled for target: {args.target}")
        try:
            tracker = TrackingController(detector, drone, args.target, args.lower_speed)
            tracker.start_tracking()
            try:
                time.sleep(30)
            except KeyboardInterrupt:
                pass
            tracker.stop_tracking()
        except Exception as e:
            print(f"Tracking error: {e}")
    else:
        try:
            while True:
                dx, dy = detector.get_offset()
                print(f"dx={dx:+d} dy={dy:+d}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("down")
            if drone:
                drone.disarm()
                drone.disconnect()


if __name__ == "__main__":
    main()
