#!/usr/bin/env python3
"""
mission/test.py — detect -> GUIDED (centered via TrackingController) -> LAND.

Start your AUTO mission yourself via RC/Mission Planner. This script only
watches, headless or windowed, and takes over once the target is confirmed.

--dry-run: no MAVLink connection at all, a stub prints what would be
commanded. Stays running past centered/lost until Ctrl+C — never lands.

--headless: skip the preview window/status overlay entirely.

Note: the periodic "is the pilot still letting us fly" check is commented
out below — ArduPilot itself stops honoring set_velo_body()/set_mode() the
instant RC override takes the vehicle out of GUIDED, regardless of whether
this script notices. That check was only ever for graceful logging/exit,
not the actual safety mechanism.
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
        log("[DryRun] connect() — no real connection made")

    def disconnect(self):
        log("[DryRun] disconnect()")

    def get_mode(self):
        return self._mode

    def set_mode(self, mode, timeout=5):
        log(f"[DryRun] set_mode({mode}) — would command this, no real FC involved")
        self._mode = mode

    def get_location(self):
        return {"lat": 0.0, "lon": 0.0, "alt": 2.0, "heading": 0.0}

    def set_velo_body(self, vx, vy, vz):
        pass


def try_set_mode(drone, mode, retries=3, delay=1.0):
    """Retries a mode change instead of letting a single failure crash the
    whole script. Returns True/False rather than raising."""
    for attempt in range(1, retries + 1):
        try:
            drone.set_mode(mode)
            return True
        except RuntimeError as e:
            log(f"set_mode('{mode}') attempt {attempt}/{retries} failed: {e}")
            if attempt < retries:
                time.sleep(delay)
    log(f"set_mode('{mode}') failed after {retries} attempts — giving up on this transition")
    return False


def parse_args():
    parser = argparse.ArgumentParser(description="Detect -> GUIDED -> center -> LAND")
    parser.add_argument("-m", "--model", default=config.MODEL_PATH)
    parser.add_argument("-l", "--labels", default=config.LABELS_PATH)
    parser.add_argument("-s", "--score_thresh", type=float, default=config.SCORE_THRESH)
    parser.add_argument("--mavlink", default=config.MAVLINK_CONNECTION)
    parser.add_argument("--baud", type=int, default=config.MAVLINK_BAUD)
    parser.add_argument("--target", required=True, help="Class name to trigger on")
    parser.add_argument("--confirm-frames", type=int, default=5,
                         help="Consecutive detections required before engaging GUIDED")
    parser.add_argument("--centered-frames", type=int, default=15,
                         help="Consecutive well-centered frames required before switching to LAND")
    parser.add_argument("--lost-timeout", type=float, default=2.0,
                         help="Seconds without a detection (while centering) before landing in place")
    parser.add_argument("--search-timeout", type=float, default=120,
                         help="Max seconds to wait for initial detection before giving up")
    parser.add_argument("--dry-run", action="store_true",
                         help="No MAVLink at all — print what would be commanded, stays running until Ctrl+C")
    parser.add_argument("--headless", action="store_true",
                         help="No preview window/status overlay. Default shows the window.")
    return parser.parse_args()


def wait_for_target(detector, drone, args):
    log(f"SEARCHING — watching for '{args.target}'. Start your AUTO mission now.")
    detector.set_status_text("SEARCHING")
    consecutive_hits = 0
    start_time = time.time()
    last_status = 0.0

    while True:
        if time.time() - start_time > args.search_timeout:
            log("SEARCHING — timeout, no detection. AUTO mission left untouched. Exiting.")
            return False

        detections = detector.get_detections()
        hit = any(d[0] == args.target for d in detections)
        consecutive_hits = consecutive_hits + 1 if hit else 0

        if time.time() - last_status > 0.5:
            mode = drone.get_mode()
            log(f"SEARCHING — target_seen={hit} streak={consecutive_hits}/{args.confirm_frames} "
                f"fc_mode={mode}")
            last_status = time.time()

        if consecutive_hits >= args.confirm_frames:
            log(f"SEARCHING — '{args.target}' confirmed over {args.confirm_frames} frames.")
            return True

        time.sleep(0.05)


def center_and_land(detector, drone, args):
    log("ENGAGING — switching to GUIDED")
    if not try_set_mode(drone, "GUIDED"):
        log("ENGAGING — could not enter GUIDED. Aborting, leaving AUTO mission untouched.")
        return
    log(f"ENGAGING — fc_mode={drone.get_mode()}")

    tracker = TrackingController(detector, drone, target_class=args.target)
    tracker.start_tracking()
    log("CENTERING — TrackingController started")

    deadzone = TrackingController.CENTER_DEADZONE
    stable_count = 0
    lost_since = None
    last_status = 0.0
    already_announced_centered = False
    outcome = None

    # last_mode_check = 0.0  # only needed if the RC-override abort check below is re-enabled

    try:
        while True:
            # ---- RC override check — commented out, see module docstring ----
            # ArduPilot itself stops honoring set_velo_body()/set_mode() the
            # instant the pilot takes manual control; this was only for
            # graceful script-side logging/exit, not the actual safety net.
            #
            # if time.time() - last_mode_check > 0.5:
            #     current_mode = drone.get_mode()
            #     last_mode_check = time.time()
            #     if current_mode != "GUIDED":
            #         log(f"CENTERING — ABORT: fc_mode={current_mode}, not GUIDED. "
            #             f"Pilot has taken manual control. Backing off, not resuming automatically.")
            #         outcome = "abort"
            #         break

            offsets = detector.get_offsets()
            offset = offsets.get(args.target)
            loc = drone.get_location()
            alt = loc["alt"] if loc else None

            if offset is None:
                if lost_since is None:
                    lost_since = time.time()
                lost_for = time.time() - lost_since
                stable_count = 0
                status = f"CENTERING — target LOST for {lost_for:.1f}s alt={alt}"
                if lost_for > args.lost_timeout:
                    if args.dry_run:
                        if lost_for - args.lost_timeout < 0.35:
                            log("CENTERING — target lost past timeout (dry-run: continuing to watch, Ctrl+C to stop)")
                    else:
                        log(f"CENTERING — target lost >{args.lost_timeout}s. Aborting to LAND in place.")
                        outcome = "lost"
                        break
            else:
                lost_since = None
                dx, dy = offset
                centered = abs(dx) <= deadzone and abs(dy) <= deadzone
                if centered:
                    stable_count += 1
                else:
                    stable_count = 0
                status = (f"CENTERING — dx={dx:+d} dy={dy:+d} centered={centered} "
                          f"stable={stable_count}/{args.centered_frames} alt={alt}")
                if stable_count >= args.centered_frames:
                    if args.dry_run:
                        if not already_announced_centered:
                            log("CENTERING — stably centered (dry-run: continuing to watch, Ctrl+C to stop)")
                            already_announced_centered = True
                    else:
                        log(f"CENTERING — stably centered for {args.centered_frames} frames.")
                        outcome = "centered"
                        break

            if time.time() - last_status > 0.3:
                log(status)
                last_status = time.time()

            time.sleep(0.05)

    finally:
        tracker.stop_tracking()
        log("CENTERING — TrackingController stopped")

    if outcome == "abort":
        return

    drone.set_velo_body(0, 0, 0)
    log("LANDING — velocity zeroed, switching to LAND")
    detector.set_status_text("LANDING")
    if not try_set_mode(drone, "LAND"):
        log("LANDING — could not confirm LAND mode. Drone velocity is zeroed; "
            "check RC/GCS manually.")
        return
    log(f"LANDING — fc_mode={drone.get_mode()}")

    if args.dry_run:
        log("LANDING — dry-run, no real descent to monitor. Done.")
        return

    log("LANDING — monitoring descent")
    last_status = 0.0
    while True:
        loc = drone.get_location()
        alt = loc["alt"] if loc else None
        if time.time() - last_status > 0.5:
            log(f"LANDING — alt={alt}")
            last_status = time.time()
        if alt is not None and alt <= 0.15:
            log("LANDING — near ground, assuming touchdown")
            break
        time.sleep(0.2)


def main():
    args = parse_args()

    log("INIT — starting detector")
    detector = ObjectDetector(
        model=args.model,
        labels=args.labels,
        score_thresh=args.score_thresh,
        headless=args.headless,
    )
    detector.start()

    if args.dry_run:
        log("INIT — DRY RUN: no MAVLink connection, using stub")
        drone = DryRunDrone()
        drone.connect()
    else:
        log(f"INIT — connecting to drone on {args.mavlink}")
        drone = Drone(args.mavlink, args.baud)
        drone.connect()
    log(f"INIT — connected, fc_mode={drone.get_mode()}")

    try:
        found = wait_for_target(detector, drone, args)
        if found:
            center_and_land(detector, drone, args)
    except KeyboardInterrupt:
        log("INTERRUPTED — attempting to land in place")
        try:
            drone.set_velo_body(0, 0, 0)
            try_set_mode(drone, "LAND")
        except Exception as e:
            log(f"INTERRUPTED — failed to switch to LAND: {e}")
    except Exception as e:
        log(f"UNEXPECTED ERROR: {e}")
        log("Attempting to land in place before exiting")
        try:
            drone.set_velo_body(0, 0, 0)
            try_set_mode(drone, "LAND")
        except Exception as land_err:
            log(f"Failed to land: {land_err}")
    finally:
        log("SHUTDOWN — stopping detector, disconnecting drone")
        detector.stop()
        drone.disconnect()
        log("SHUTDOWN — done")


if __name__ == "__main__":
    main()