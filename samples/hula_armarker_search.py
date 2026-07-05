import sys
import os
import time
import threading

sys.path.insert(0, r"C:\oit\home\ipbl")  # 共有の my_libs パッケージをインポートできるようにする

import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture

CRUISE_CM       = 50   # altitude gain (cm) commanded right after takeoff
SEARCH_DEG      = 10   # degrees turned per search rotation step
SEARCH_INTERVAL = 1.2  # seconds between search rotation attempts

# ── Non-blocking execution ─────────────────────────────────────────────────
# pyhula flight commands (e.g. single_fly_turnleft) block until the drone
# reports completion. Repeating one every SEARCH_INTERVAL directly inside
# the main loop would freeze frame capture and keyboard polling for that
# long each time, so it is dispatched to a daemon thread instead. The lock
# makes a still-busy command drop the next request rather than queue it.
_flight_lock = threading.Lock()

def run_flight(fn, *args):
    def _t():
        if not _flight_lock.acquire(blocking=False):
            return
        try:
            fn(*args)
        finally:
            _flight_lock.release()
    threading.Thread(target=_t, daemon=True).start()

def main():
    # 1. Connect first
    try:
        api = pyhula.UserApi()
        print("Connecting to drone...")
        api.connect()
        time.sleep(1.0)
    except Exception as e:
        print(f"[ERROR] Failed to setup drone: {e}")
        sys.exit(1)
    api.single_fly_barrier_aircraft(False)  # disable the proximity-avoidance geofence

    # 2. Activate watcher and open stream
    with SafeDroneWatcher(api):
        cap = VideoCapture(api)

        def rotate(deg):
            """Turn in place. Positive deg = turn left, negative deg = turn right."""
            if deg < 0:
                api.single_fly_turnright(-deg)
            else:
                api.single_fly_turnleft(deg)

        is_airborne = False
        last_search = 0.0

        print("Video stream active.")
        print(">>> TO TAKE OFF  : Press 'f' inside the video window <<<")
        print(">>> TO QUIT      : Press 'q' or [Ctrl+C] <<<")

        while cap.isOpened():
            ret, frame = cap.read()

            key_press = cv2.waitKey(1) & 0xFF
            if key_press == ord('q'):
                print("[INTERRUPT] 'q' pressed.")
                break
            if not ret or frame is None:
                continue

            # --- Takeoff on 'f' key ---
            if not is_airborne:
                if key_press == ord('f'):
                    print("--- Starting Takeoff ---")
                    api.single_fly_takeoff()
                    api.single_fly_up(CRUISE_CM)
                    is_airborne = True
                    last_search = time.time()
                    run_flight(rotate, -SEARCH_DEG)  # kick off the first search turn right away
                    print("[SEARCH] Airborne. Rotating to search...")
                    continue
                else:
                    cv2.putText(frame, "STANDBY ON GROUND | Press 'f' to Takeoff",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imshow("Hula Camera", frame)
                    continue

            # --- Continuous search rotation ---
            now = time.time()
            if now - last_search >= SEARCH_INTERVAL:
                run_flight(rotate, -SEARCH_DEG)
                last_search = now

            cv2.putText(frame, "State: SEARCH", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
            cv2.imshow("Hula Camera", frame)

        # --- Cleanup ---
        # Wait (with a bounded timeout) for any rotate command a background
        # run_flight() thread might still be mid-call on, so it can't race
        # with single_fly_touchdown() on the same api object. A timeout is
        # used instead of an unbounded wait so that a stuck SDK call on the
        # background thread can never prevent shutdown -- touchdown is sent
        # regardless, even if the wait times out.
        acquired = _flight_lock.acquire(timeout=3.0)
        if not acquired:
            print("[WARNING] A flight command is still in progress after 3s; sending touchdown anyway.")
        try:
            print("Sending safe touchdown command...")
            api.single_fly_touchdown()
            print("Touchdown command returned.")
        finally:
            if acquired:
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