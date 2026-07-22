import sys
import os
import time
import threading
from enum import Enum, auto

sys.path.insert(0, r"C:\oit\home\ipbl")  # allow importing shared my_libs package

import numpy as np
import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture
from my_libs.detection_timer import DetectionTimer

# ── Tuning parameters ──────────────────────────────────────────────────────
DRONE_IP         = "192.168.100.XXX"  # set to your drone's address (see drone_control.md Step 0)
CRUISE_CM        = 50     # altitude gain (cm) commanded right after takeoff
TARGET_MARKER_PX = 150    # desired marker diagonal size (px); proxy for "distance to marker" -- calibrate per marker size/camera
MOVE_STEP        = 10     # max cm per forward/back command while adjusting distance
SEARCH_DEG       = 10     # degrees turned per search/re-center rotation step
SEARCH_INTERVAL  = 1.2    # seconds between rotation attempts while searching (State.SEARCH)
STAY_MS          = 1000.0 # time (ms) the marker must stay locked on (centered + correct distance) before landing
STAY_GRACE_MS    = 300.0  # allowed gap (ms) where the lock briefly drops without resetting the timer
DEAD_BAND        = 0.15   # normalized error tolerance below which no correction is sent (avoids jitter)
CMD_INTERVAL     = 0.25   # minimum seconds between flight commands sent via send_cmd (rate limiting)

# ArUco marker detector setup.
ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

class State(Enum):
    SEARCH = auto()  # no marker tracked yet: rotate slowly, scanning for one
    FOLLOW = auto()  # marker tracked: keep it centered and at target distance
    LAND   = auto()  # marker locked on long enough: exit the loop and land

# ── Non-blocking execution ─────────────────────────────────────────────────
# pyhula flight/LED calls block, so route them through daemon threads with a
# drop-if-busy lock so the video loop keeps reading frames at full rate. Two
# separate locks are used so an LED update never blocks a pending flight
# command, or vice versa.
_flight_lock = threading.Lock()
_led_lock    = threading.Lock()

def run_flight(fn, *args):
    def _t():
        if not _flight_lock.acquire(blocking=False):
            return  # a previous flight command is still executing; skip this one
        try:
            fn(*args)
        finally:
            _flight_lock.release()
    threading.Thread(target=_t, daemon=True).start()

def run_led(fn, *args):
    def _t():
        if not _led_lock.acquire(blocking=False):
            return  # a previous LED command is still executing; skip this one
        try:
            fn(*args)
        finally:
            _led_lock.release()
    threading.Thread(target=_t, daemon=True).start()

# ── Utilities ──────────────────────────────────────────────────────────────
def marker_metrics(corners, idx, frame_w):
    """Return (normalized_center_x, diagonal_size_px) for the marker at corners[idx].

    corners[idx] has shape (1, 4, 2): the marker's 4 corners, ordered
    top-left, top-right, bottom-right, bottom-left. cx is normalized to
    [0, 1] across the frame width so it can be compared directly against
    the 0.5 (frame center) target regardless of resolution. diag_px is the
    pixel distance between the top-left and bottom-right corners -- a
    rotation-robust proxy for how close the marker is to the camera (same
    corner[0]/corner[2] technique as the reference AR-marker material).
    """
    pts = corners[idx][0]
    cx = float(pts[:, 0].mean()) / frame_w
    diag_px = float(np.linalg.norm(pts[2] - pts[0]))
    return cx, diag_px

def clamp_int(v, lo, hi):
    """Clamp v into [lo, hi] and truncate to int (used to bound command magnitudes)."""
    return int(max(lo, min(hi, v)))

# ── Main ───────────────────────────────────────────────────────────────────
def main():
    # 1. Connect first
    try:
        api = pyhula.UserApi()
        print(f"Connecting to drone at {DRONE_IP}...")
        api.connect(DRONE_IP)
        time.sleep(3.0)
    except Exception as e:
        print(f"[ERROR] Failed to setup drone: {e}")
        sys.exit(1)
    api.single_fly_barrier_aircraft(False)  # disable the "aircraft barrier" safety geofence for this single drone

    # 2. Activate watcher and open stream
    with SafeDroneWatcher(api):
        cap = VideoCapture(api)

        state         = State.SEARCH
        stay_timer    = DetectionTimer(STAY_MS, STAY_GRACE_MS)
        is_airborne   = False
        last_search   = 0.0   # wall-clock time of the last search-rotation command
        last_cmd_time = 0.0   # wall-clock time of the last rate-limited flight command (see send_cmd)
        prev_led      = -1    # last LED state sent, so we only re-send the LED color when it actually changes

        def send_cmd(fn, *args):
            """Rate-limited wrapper around run_flight: drops the call if CMD_INTERVAL hasn't elapsed yet."""
            nonlocal last_cmd_time
            if time.time() - last_cmd_time < CMD_INTERVAL:
                return
            last_cmd_time = time.time()
            run_flight(fn, *args)

        def rotate(deg):
            """Turn in place. Positive deg = turn left, negative deg = turn right."""
            if deg < 0:
                api.single_fly_turnright(-deg)
            else:
                api.single_fly_turnleft(deg)

        def reset_stay():
            """Clear the marker-lock hold timer, e.g. when the tracked marker is lost."""
            stay_timer.reset()

        print("Video stream active.")
        print(">>> TO TAKE OFF  : Press 'f' inside the video window <<<")
        print(">>> TO QUIT      : Press 'q' or [Ctrl+C] <<<")
        print("  Show marker         → Follow")
        print("  Hold centered 1s    → Land")

        while cap.isOpened():
            ret, frame = cap.read()

            key_press = cv2.waitKey(1) & 0xFF
            if key_press == ord('q'):
                print("[INTERRUPT] 'q' pressed.")
                break
            if not ret or frame is None:
                continue

            # Timestamp from the video stream itself (not wall-clock time),
            # so the lock-hold timer stays consistent even if frame delivery
            # stutters or the stream has its own internal clock.
            current_msec = cap.get(cv2.CAP_PROP_POS_MSEC)

            # --- Takeoff on 'f' key ---
            if not is_airborne:
                if key_press == ord('f'):
                    print("--- Starting Takeoff ---")
                    api.single_fly_takeoff()
                    api.single_fly_up(CRUISE_CM)
                    is_airborne   = True
                    last_search   = time.time()
                    last_cmd_time = 0.0  # allow the very next send_cmd call through immediately
                    run_flight(rotate, -SEARCH_DEG)  # kick off the first search turn right away
                    print("[SEARCH] Airborne. Searching for marker...")
                    continue
                else:
                    # Still on the ground: just show a standby prompt, skip all detection/flight logic
                    cv2.putText(frame, "STANDBY ON GROUND | Press 'f' to Takeoff",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imshow("Hula Camera", frame)
                    continue

            # --- Marker detection ---
            now = time.time()
            _, w = frame.shape[:2]
            corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICTIONARY)

            # Only ever follow the first detected marker; ignore any others in view.
            target_idx = 0 if ids is not None and len(ids) > 0 else None

            x_err = size_err = None
            is_locked = False
            if target_idx is not None:
                cx, diag_px = marker_metrics(corners, target_idx, w)
                x_err     = cx - 0.5  # horizontal offset from frame center, normalized [-0.5, 0.5]
                size_err  = (diag_px - TARGET_MARKER_PX) / TARGET_MARKER_PX  # positive = marker looks bigger than target (too close)
                is_locked = abs(x_err) <= DEAD_BAND and abs(size_err) <= DEAD_BAND

            # --- LED feedback --- (0 = no marker: off, 1 = tracking: green, 2 = locked on while following: red)
            if target_idx is None:
                led = 0
            elif is_locked and state == State.FOLLOW:
                led = 2
            else:
                led = 1

            if led != prev_led:  # only issue a lamp command when the color actually needs to change
                if led == 0:
                    run_led(api.single_fly_lamplight, 0,   0,   0, 1,  2)
                elif led == 1:
                    run_led(api.single_fly_lamplight, 0, 255,   0, 2, 32)
                else:
                    run_led(api.single_fly_lamplight, 255,  0,  0, 2, 32)
                prev_led = led

            # --- Stay timer (stream-clock driven) ---
            # Only counts toward landing while actively in FOLLOW state, so a
            # brief lock achieved during SEARCH doesn't land the drone.
            stay_reached = stay_timer.update(
                is_locked and state == State.FOLLOW, current_msec)

            # Elapsed hold time, for the on-screen progress readout only (not used by the state machine)
            stay_prog_s = 0.0
            if stay_timer.start_time is not None and not stay_timer.is_reached:
                stay_prog_s = min(current_msec - stay_timer.start_time,
                                    STAY_MS) / 1000.0

            # --- State machine ---
            if state == State.SEARCH:
                if target_idx is not None:
                    # A marker just appeared: stop the rotation immediately (hover)
                    # and do one small corrective turn toward it before switching to FOLLOW.
                    run_flight(api.single_fly_hover_flight, 0.5)
                    if x_err > DEAD_BAND:
                        run_flight(rotate, -clamp_int(x_err * 30, 1, SEARCH_DEG))
                    elif x_err < -DEAD_BAND:
                        run_flight(rotate, clamp_int(-x_err * 30, 1, SEARCH_DEG))
                    state = State.FOLLOW
                    reset_stay()
                    print("[FOLLOW] Marker acquired — following.")

                elif now - last_search >= SEARCH_INTERVAL:
                    # No marker yet: keep turning the same direction at a fixed
                    # cadence (SEARCH_INTERVAL) until one comes into view.
                    send_cmd(rotate, -SEARCH_DEG)
                    last_search = now

            elif state == State.FOLLOW:
                if stay_reached:
                    print("[LAND] Marker held centered — landing.")
                    state = State.LAND
                    break  # exit the video loop; cleanup below sends the touchdown command

                if target_idx is None:
                    # Lost the marker mid-follow: fall back to SEARCH and reset any partial lock hold
                    state = State.SEARCH
                    print("[SEARCH] Marker lost — resuming search.")
                    send_cmd(rotate, -SEARCH_DEG)
                    last_search = now
                    reset_stay()
                else:
                    # Priority: recenter horizontally first; only adjust
                    # forward/back distance once roughly centered, so the
                    # drone doesn't drift sideways while approaching/retreating.
                    if x_err > DEAD_BAND:
                        send_cmd(rotate, -clamp_int(x_err * 40, 1, MOVE_STEP))
                    elif x_err < -DEAD_BAND:
                        send_cmd(rotate, clamp_int(-x_err * 40, 1, MOVE_STEP))
                    elif abs(size_err) > DEAD_BAND:
                        if size_err > 0:
                            # Marker appears larger than target size → drone is too close → back away
                            send_cmd(api.single_fly_back, clamp_int(size_err * 40, 1, MOVE_STEP))
                        else:
                            # Marker appears smaller than target size → drone is too far → move closer
                            send_cmd(api.single_fly_forward, clamp_int(-size_err * 40, 1, MOVE_STEP))

            # --- Render overlay ---
            annotated = frame.copy()
            if target_idx is not None:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            lc = {State.SEARCH: (0, 200, 255),
                    State.FOLLOW: (0, 255,   0),
                    State.LAND:   (0,   0, 255)}.get(state, (255, 255, 255))
            cv2.putText(annotated, f"State: {state.name}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, lc, 2)
            if stay_prog_s > 0:
                cv2.putText(annotated,
                            f"Lock: {stay_prog_s:.1f}/{STAY_MS/1000:.1f}s",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 100, 255), 2)
            cv2.imshow("Hula Camera", annotated)

        # --- Cleanup ---
        # Runs on 'q', on State.LAND (lock-confirmed landing), or on stream failure.
        # Wait (with a bounded timeout) for any flight/LED command a
        # background thread might still be mid-call on, so it can't race
        # with single_fly_touchdown() on the same api object -- SEARCH's
        # continuous rotation makes this likely if 'q' is pressed while no
        # marker has been found yet. A timeout is used instead of an
        # unbounded wait so a stuck SDK call on a background thread can
        # never prevent shutdown -- touchdown is sent regardless, even if
        # the wait times out.
        flight_acquired = _flight_lock.acquire(timeout=3.0)
        led_acquired    = _led_lock.acquire(timeout=3.0)
        if not (flight_acquired and led_acquired):
            print("[WARNING] A flight/LED command is still in progress after 3s; sending touchdown anyway.")
        try:
            try:
                api.single_fly_lamplight(0, 0, 0, 1, 2)  # turn the LED off
            except Exception:
                pass  # LED failure shouldn't prevent the landing below
            print("Sending safe touchdown command...")
            api.single_fly_touchdown()  # blocks synchronously
            print("Touchdown command returned.")
        finally:
            if led_acquired:
                _led_lock.release()
            if flight_acquired:
                _flight_lock.release()

        cap.release()
        print("cap.release() done.")
        print("Resources released successfully.")

    # cv2.destroyAllWindows() is intentionally skipped here: once the video
    # loop has stopped calling cv2.waitKey(), destroyAllWindows() can hang
    # indefinitely waiting on the HighGUI window's message loop on Windows
    # (confirmed by testing -- execution stops right after cap.release()).
    # The process is about to force-exit anyway, and the OS reclaims the
    # window along with everything else on exit, so there is nothing to
    # gain from waiting on a graceful window-close that may never return.
    #
    # pyhula's internal RTP/video receiver thread(s) also keep running once
    # started, and VideoCapture.release() in "hula" mode never actually
    # tells them to stop -- it only flips a Python-side flag. If that
    # thread isn't a daemon thread, normal interpreter shutdown can hang
    # waiting for it to join. All flight-safety-critical work (touchdown)
    # is already complete at this point, so force-exit the process
    # immediately instead of waiting.
    os._exit(0)

if __name__ == "__main__":
    main()