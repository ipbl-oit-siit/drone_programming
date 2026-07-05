import sys
import os
import time
import threading

sys.path.insert(0, r"C:\oit\home\ipbl")  # 共有の my_libs パッケージをインポートできるようにする

import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture

CRUISE_CM       = 50
SEARCH_DEG      = 10
SEARCH_INTERVAL = 1.2

# ArUco marker detector setup.
ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ── Non-blocking execution ─────────────────────────────────────────────────
# Two separate locks: flight commands and LED commands are independent SDK
# calls, so an LED update should never block a pending rotation (or vice versa).
_flight_lock = threading.Lock()
_led_lock    = threading.Lock()

def run_flight(fn, *args):
    def _t():
        if not _flight_lock.acquire(blocking=False):
            return
        try:
            fn(*args)
        finally:
            _flight_lock.release()
    threading.Thread(target=_t, daemon=True).start()

def run_led(fn, *args):
    def _t():
        if not _led_lock.acquire(blocking=False):
            return
        try:
            fn(*args)
        finally:
            _led_lock.release()
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
    api.single_fly_barrier_aircraft(False)

    # 2. Activate watcher and open stream
    with SafeDroneWatcher(api):
        cap = VideoCapture(api)

        def rotate(deg):
            if deg < 0:
                api.single_fly_turnright(-deg)
            else:
                api.single_fly_turnleft(deg)

        is_airborne = False
        is_staying  = False   # True while a marker is currently locked on and the drone is hovering
        last_search = 0.0
        prev_led    = -1

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
                    run_flight(rotate, -SEARCH_DEG)
                    print("[SEARCH] Airborne. Searching for marker...")
                    continue
                else:
                    cv2.putText(frame, "STANDBY ON GROUND | Press 'f' to Takeoff",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imshow("Hula Camera", frame)
                    continue

            # --- Marker detection ---
            now = time.time()
            corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICTIONARY)
            marker_found = ids is not None and len(ids) > 0

            # --- State transitions ---
            if marker_found and not is_staying:
                is_staying = True
                run_flight(api.single_fly_hover_flight, 0.5)
                print("[STAY] Marker acquired — holding position.")
            elif not marker_found and is_staying:
                is_staying = False
                last_search = now
                run_flight(rotate, -SEARCH_DEG)
                print("[SEARCH] Marker lost — resuming search.")
            elif not is_staying and now - last_search >= SEARCH_INTERVAL:
                run_flight(rotate, -SEARCH_DEG)
                last_search = now

            # --- LED feedback --- (0 = no marker: off, 1 = marker held: green)
            led = 1 if is_staying else 0
            if led != prev_led:
                if led == 0:
                    run_led(api.single_fly_lamplight, 0, 0, 0, 1, 2)
                else:
                    run_led(api.single_fly_lamplight, 0, 255, 0, 2, 32)
                prev_led = led

            # --- Render overlay ---
            annotated = frame.copy()
            if marker_found:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            label = "STAY" if is_staying else "SEARCH"
            color = (0, 255, 0) if is_staying else (0, 200, 255)
            cv2.putText(annotated, f"State: {label}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow("Hula Camera", annotated)

        # --- Cleanup ---
        # Wait (with a bounded timeout) for any flight/LED command a
        # background thread might still be mid-call on, so it can't race
        # with single_fly_touchdown() on the same api object. A timeout is
        # used instead of an unbounded wait so that a stuck SDK call on a
        # background thread can never prevent shutdown -- touchdown is sent
        # regardless, even if the wait times out.
        flight_acquired = _flight_lock.acquire(timeout=3.0)
        led_acquired    = _led_lock.acquire(timeout=3.0)
        if not (flight_acquired and led_acquired):
            print("[WARNING] A flight/LED command is still in progress after 3s; sending touchdown anyway.")
        try:
            try:
                api.single_fly_lamplight(0, 0, 0, 1, 2)
            except Exception:
                pass
            print("Sending safe touchdown command...")
            api.single_fly_touchdown()
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