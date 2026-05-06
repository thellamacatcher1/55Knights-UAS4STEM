import argparse
import threading

import config
from uas_vision import ObjectDetector

def parse_args():
    parser = argparse.ArgumentParser(description="Drone Vision + MAVLink")
    parser.add_argument("-m", "--model",       default=config.MODEL_PATH)
    parser.add_argument("-l", "--labels",      default=config.LABELS_PATH)
    parser.add_argument("-s", "--score_thresh",type=float, default=config.SCORE_THRESH)
    parser.add_argument("--mavlink",           default=config.MAVLINK_CONNECTION)
    return parser.parse_args()

def main():
    args = parse_args()

    detector = ObjectDetector(
        model=args.model,
        labels=args.labels,
        score_thresh=args.score_thresh,
    )
    detector.start()

    vision_thread = threading.Thread(target=detector.run_loop, daemon=True)
    vision_thread.start()

    try:
        vision_thread.join()
    except KeyboardInterrupt:
        print("off")

if __name__ == "__main__":
    main()