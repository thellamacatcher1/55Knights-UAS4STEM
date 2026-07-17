#!/usr/bin/env python3
"""
mission/test.py - detect, switch to guided, center + precision descend with
TrackingController, then land.

start your AUTO mission yourself thru RC or Mission Planner. this script just
watches (headless or windowed) and takes over once target is confirmed.

if target gets lost mid centering/descent (not landed yet) the script wont
force a land in place anymore, it goes back to whatever mode was active
before guided (usually AUTO) so the original mission can keep going.

--dry-run: no mavlink connection at all, stub just prints what would be sent.
keeps running til ctrl+c, never actually reaches LAND.

--headless: skip the preview window and overlay entirely.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from uas_vision import ObjectDetector
from uas_mavlink import Drone
from uas_mavlink.controller import TrackingController


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")


class DryRunDrone:
    def __init__(self):
        self._mode = "AUTO"

    def connect(self):
        log("[DryRun] connect() - not really connecting to anything")

    def disconnect(self):
        log("[DryRun] disconnect()")

    def get_mode(self):
        return self._mode

    def set_mode(self, mode, timeout=5):
        log(f"[DryRun] set_mode({mode}) - would send this, no real fc")
        self._mode = mode

    def get_location(self):
        return {"lat": 0.0, "lon": 0.0, "alt": 2.0, "heading": 0.0}

    def get_velocity(self):
        return {"vx": 0.0, "vy": 0.0, "vz": 0.0}

    def brake_and_wait(self, threshold=0.3, timeout=10):
        log("[DryRun] brake_and_wait() - no real fc, pretending we already stopped")
        return True

    def set_velo_body(self, vx, vy, vz):
        pass


def try_set_mode(drone, mode, retries=3, delay=1.0):
    # retries a mode change a few times instead of just crashing the whole script
    for attempt in range(1, retries + 1):
        try:
            drone.set_mode(mode)
            return True
        except RuntimeError as e:
            log(f"set_mode('{mode}') attempt {attempt}/{retries} failed: {e}")
            if attempt < retries:
                time.sleep(delay)
    log(f"set_mode('{mode}') failed after {retries} tries, giving up on this one")
    return False


def parse_args():
    parser = argparse.ArgumentParser(description="Detect -> GUIDED -> center -> precision descend -> LAND")
    parser.add_argument("-m", "--model", default=config.MODEL_PATH)
    parser.add_argument("-l", "--labels", default=config.LABELS_PATH)
    parser.add_argument("-s", "--score_thresh", type=float, default=config.SCORE_THRESH)
    parser.add_argument("--mavlink", default=config.MAVLINK_CONNECTION)
    parser.add_argument("--baud", type=int, default=config.MAVLINK_BAUD)
    parser.add_argument("--target", required=True, help="class name to trigger on")
    parser.add_argument("--confirm-frames", type=int, default=5,
                         help="how many frames in a row before we switch to guided")
    parser.add_argument("--centered-frames", type=int, default=15,
                         help="how many frames in a row centered before we start descending")
    parser.add_argument("--lost-timeout", type=float, default=2.0,
                         help="seconds with no detection before going back to old flight mode")
    parser.add_argument("--search-timeout", type=float, default=120,
                         help="max seconds to wait for first detection before giving up")
    parser.add_argument("--dry-run", action="store_true",
                         help="no mavlink at all, just print what would be sent, runs til ctrl+c")
    parser.add_argument("--headless", action="store_true",
                         help="no preview window/overlay. default is window shown")
    return parser.parse_args()


def wait_for_target(detector, drone, args):
    log(f"SEARCHING - watching for '{args.target}'. go ahead and start your AUTO mission now.")
    detector.set_status_text("SEARCHING")
    consecutive_hits = 0
    start_time = time.time()
    last_status = 0.0

    while True:
        if time.time() - start_time > args.search_timeout:
            log("SEARCHING - timed out, never saw it. AUTO mission left alone. exiting.")
            return False

        detections = detector.get_detections()
        hit = any(d[0] == args.target for d in detections)
        consecutive_hits = consecutive_hits + 1 if hit else 0

        if time.time() - last_status > 0.5:
            mode = drone.get_mode()
            log(f"SEARCHING - target_seen={hit} streak={consecutive_hits}/{args.confirm_frames} "
                f"fc_mode={mode}")
            last_status = time.time()

        if consecutive_hits >= args.confirm_frames:
            log(f"SEARCHING - '{args.target}' confirmed over {args.confirm_frames} frames.")
            return True

        time.sleep(0.05)


def center_and_land(detector, drone, args):
    previous_mode = drone.get_mode()
    log(f"ENGAGING - previous flight mode was '{previous_mode}', we'll go back to this if target is lost")

    log("ENGAGING - braking to a stop before we start centering")
    drone.brake_and_wait(threshold=0.3, timeout=10)

    log("ENGAGING - switching to GUIDED")
    if not try_set_mode(drone, "GUIDED"):
        log("ENGAGING - couldn't get into GUIDED. bailing, AUTO mission untouched.")
        return
    log(f"ENGAGING - fc_mode={drone.get_mode()}")

    tracker = TrackingController(detector, drone, target_class=args.target)
    tracker.start_tracking()
    log("CENTERING - TrackingController started")

    deadzone = TrackingController.CENTER_DEADZONE
    stable_count = 0
    lost_since = None
    last_status = 0.0
    phase = "centering"
    outcome = None

    try:
        while True:
            offsets = detector.get_offsets()
            offset = offsets.get(args.target)
            loc = drone.get_location()
            alt = loc["alt"] if loc else None

            if offset is None:
                if lost_since is None:
                    lost_since = time.time()
                lost_for = time.time() - lost_since
                stable_count = 0
                status = f"{phase.upper()} - target LOST for {lost_for:.1f}s alt={alt}"
                if lost_for > args.lost_timeout:
                    if args.dry_run:
                        if lost_for - args.lost_timeout < 0.35:
                            log(f"{phase.upper()} - lost past timeout (dry run, keeps going, ctrl+c to stop)")
                    else:
                        log(f"{phase.upper()} - lost for over {args.lost_timeout}s. "
                            f"going back to '{previous_mode}'.")
                        outcome = "lost"
                        break
            else:
                lost_since = None
                dx, dy = offset
                centered = abs(dx) <= deadzone and abs(dy) <= deadzone

                if phase == "centering":
                    if centered:
                        stable_count += 1
                    else:
                        stable_count = 0
                    status = (f"CENTERING - dx={dx:+d} dy={dy:+d} centered={centered} "
                              f"stable={stable_count}/{args.centered_frames} alt={alt}")
                    if stable_count >= args.centered_frames:
                        log(f"CENTERING - centered for {args.centered_frames} frames straight. starting descent.")
                        phase = "descending"

                elif phase == "descending":
                    status = f"DESCENDING - dx={dx:+d} dy={dy:+d} centered={centered} alt={alt}"
                    if not args.dry_run and tracker.ready_for_final_land():
                        log(f"DESCENDING - alt={alt}, low enough now. handing off to LAND.")
                        outcome = "descended"
                        break

            if time.time() - last_status > 0.3:
                log(status)
                last_status = time.time()

            time.sleep(0.05)

    finally:
        tracker.stop_tracking()
        log("CENTERING - TrackingController stopped")

    if outcome == "lost":
        drone.set_velo_body(0, 0, 0)
        log(f"REVERTING - velocity zeroed, switching back to '{previous_mode}'")
        if not try_set_mode(drone, previous_mode):
            log(f"REVERTING - couldn't confirm '{previous_mode}'. velocity is zeroed at least, "
                f"check rc/gcs manually.")
        else:
            log(f"REVERTING - fc_mode={drone.get_mode()}")
        return

    if outcome != "descended":
        return

    drone.set_velo_body(0, 0, 0)
    log("LANDING - velocity zeroed, switching to LAND")
    detector.set_status_text("LANDING")
    if not try_set_mode(drone, "LAND"):
        log("LANDING - couldn't confirm LAND mode. velocity is zeroed, check rc/gcs manually.")
        return
    log(f"LANDING - fc_mode={drone.get_mode()}")

    if args.dry_run:
        log("LANDING - dry run, nothing real to monitor. done.")
        return

    log("LANDING - watching for touchdown")
    last_status = 0.0
    while True:
        loc = drone.get_location()
        alt = loc["alt"] if loc else None
        if time.time() - last_status > 0.5:
            log(f"LANDING - alt={alt}")
            last_status = time.time()
        if alt is not None and alt <= 0.15:
            log("LANDING - close to ground, assuming touchdown")
            break
        time.sleep(0.2)


def main():
    args = parse_args()

    log("INIT - starting detector")
    detector = ObjectDetector(
        model=args.model,
        labels=args.labels,
        score_thresh=args.score_thresh,
        headless=args.headless,
    )
    detector.start()

    if args.dry_run:
        log("INIT - dry run, no mavlink, using stub drone")
        drone = DryRunDrone()
        drone.connect()
    else:
        log(f"INIT - connecting to drone on {args.mavlink}")
        drone = Drone(args.mavlink, args.baud)
        drone.connect()
    log(f"INIT - connected, fc_mode={drone.get_mode()}")

    try:
        found = wait_for_target(detector, drone, args)
        if found:
            center_and_land(detector, drone, args)
    except KeyboardInterrupt:
        log("INTERRUPTED - trying to land in place")
        try:
            drone.set_velo_body(0, 0, 0)
            try_set_mode(drone, "LAND")
        except Exception as e:
            log(f"INTERRUPTED - failed to switch to LAND: {e}")
    except Exception as e:
        log(f"UNEXPECTED ERROR: {e}")
        log("trying to land in place before exiting")
        try:
            drone.set_velo_body(0, 0, 0)
            try_set_mode(drone, "LAND")
        except Exception as land_err:
            log(f"failed to land: {land_err}")
    finally:
        log("SHUTDOWN - stopping detector, disconnecting drone")
        detector.stop()
        drone.disconnect()
        log("SHUTDOWN - done")


if __name__ == "__main__":
    main()