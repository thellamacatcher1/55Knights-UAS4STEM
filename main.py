import argparse
import threading
import time

import config
from uas_vision import ObjectDetector
from uas_mavlink import Drone

def parse_args():
    parser = argparse.ArgumentParser(description="Drone Vision + MAVLink")
    parser.add_argument("-m", "--model",        default=config.MODEL_PATH)
    parser.add_argument("-l", "--labels",       default=config.LABELS_PATH)
    parser.add_argument("-s", "--score_thresh", type=float, default=config.SCORE_THRESH)
    parser.add_argument("--mavlink", default=config.MAVLINK_CONNECTION)
    parser.add_argument("--baud",    type=int, default=config.MAVLINK_BAUD)
    return parser.parse_args()

def main():
    args = parse_args()

    detector = ObjectDetector(
        model=args.model,
        labels=args.labels,
        score_thresh=args.score_thresh,
    )
    detector.start()
    
    """
    drone = Drone(args.mavlink, args.baud)
    drone.connect()
    """

    vision_thread = threading.Thread(target=detector.run_loop, daemon=True)
    vision_thread.start()
    try:
        while True:
            dx, dy = detector.get_offset()
            print(f"dx={dx:+d} dy={dy:+d}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("down")
        
        """
        drone.disarm()
        drone.disconnect()
        """

if __name__ == "__main__":
    main()