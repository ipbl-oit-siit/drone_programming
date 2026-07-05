# AR Marker Detection and Search/Stay/Follow Drone Control

[back to the top page](../README.md)

---

## Objectives
- This page teaches the fundamentals of **state-machine-based drone control**: searching for a target, recognizing it, and controlling direction and distance toward it — one capability at a time.
- An **ArUco marker** is used as the target because OpenCV detects it deterministically (no confidence scores, no missed frames from a classifier), so the focus stays entirely on the flight-control logic itself.
- Four practices build the program up one capability at a time:

  | Step | New capability | No-flight? |
  |---|---|---|
  | 1 | Takeoff + continuous rotation (`SEARCH`) | No |
  | 2 | Marker detection → stop + LED (`SEARCH` ⇄ `STAY`) | No |
  | 3 | Distance/position estimation only (near/appropriate/far, left/center/right) | **Yes** |
  | 4 | Full `SEARCH` / `FOLLOW` / `LAND` state machine with a hold-timer landing trigger | No |

## Prerequisite
- All practices in [drone_control.md](https://github.com/ipbl-oit-siit/drone_programming/blob/main/docs/drone_control.md) have been completed, especially `hover_and_failsafe_test` and `camera_angle_control`.
- **Drone IP**: every program below connects with a fixed `DRONE_IP` constant, exactly like [drone_control.md](https://github.com/ipbl-oit-siit/drone_programming/blob/main/docs/drone_control.md). Follow that page's "Step 0: How to Check the Drone IP Address (Windows)" to find your address and set `DRONE_IP` accordingly — not re-explained here.
- **OpenCV version note**: this environment's OpenCV predates 4.7, so `cv2.aruco.detectMarkers(img, dictionary)` — the free function shown in the official AR-marker reference material ([basics_armarker_static.md](https://github.com/ipbl-oit-siit/drone_programming/blob/main/docs/basics_armarker_static.md)) — is used directly, exactly as written there. See the "ArUco Marker Detection API" note below if you ever need to port this to OpenCV ≥ 4.7, where that function was replaced by the `cv2.aruco.ArucoDetector` class.
- **A printed marker**: you need at least one physical marker from the `DICT_4X4_50` dictionary. Generate and print one with:
  ```python
  import cv2
  dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
  marker_img = cv2.aruco.generateImageMarker(dictionary, 0, 2000)  # marker ID 0, 200x200 px
  cv2.imwrite("marker_id0.png", marker_img)
  ```
  Print `marker_id0.png` at a size of roughly 10–15 cm per side and mount it on a flat, rigid surface (cardboard is fine). Any ID from 0–49 works — the programs below do not check which ID was detected.

---

## :green_square: Architecture: Extending the GitHub Base Programs

Every practice on this page follows the same skeleton as [drone_control.md](https://github.com/ipbl-oit-siit/drone_programming/blob/main/docs/drone_control.md):

```python
with SafeDroneWatcher(api):
    cap = VideoCapture(api)          # handles RTP init internally

    while cap.isOpened():
        ret, frame = cap.read()
        key_press = cv2.waitKey(1) & 0xFF   # ONE waitKey per loop pass
        if key_press == ord('q'):
            break
        if not ret or frame is None:
            continue
        ...
```

This page adds three layers on top, introduced one at a time:

| Layer | What it adds | First introduced |
|---|---|---|
| **`cv2.aruco.detectMarkers()`** | `cv2.aruco.detectMarkers(frame, ARUCO_DICTIONARY)` → `corners`, `ids` inside the loop | Step 2 |
| **`run_flight()` / `run_led()`** | Non-blocking dispatch of flight and LED commands via daemon threads | Step 1 (flight), Step 2 (LED) |
| **`DetectionTimer`** | Requires the marker to stay locked on for a continuous duration before landing | Step 4 |

---

## :green_square: ArUco Marker Detection API

```python
ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

corners, ids, rejected = cv2.aruco.detectMarkers(frame, ARUCO_DICTIONARY)
```

| Item | Type / Shape | Meaning |
|---|---|---|
| `ARUCO_DICTIONARY` | `cv2.aruco.Dictionary` | `DICT_4X4_50` = a 4×4-bit pattern dictionary with 50 unique IDs (0–49). |
| `corners` | `tuple` of `numpy.ndarray`, each `(1, 4, 2)` | One entry per detected marker. The 4 points are the corner pixel coordinates in the order **top-left, top-right, bottom-right, bottom-left**. |
| `ids` | `numpy.ndarray` of shape `(N, 1)`, or `None` | The decoded marker ID for each entry in `corners`. **`None` (not an empty array) when nothing is detected** — always check `ids is not None` before indexing. |
| `rejected` | `list` | Candidate shapes that looked marker-like but failed to decode. Not used on this page. |

> [!NOTE]
> ### OpenCV version compatibility
> `cv2.aruco.detectMarkers(image, dictionary)` is the **free-function** form of ArUco detection, matching the AR-marker reference material ([basics_armarker_static.md](https://github.com/ipbl-oit-siit/drone_programming/blob/main/docs/basics_armarker_static.md)) exactly. This environment's OpenCV predates 4.7, so this is the form that actually works here.
> If you ever run this code on OpenCV **≥ 4.7**, this free function no longer exists (`AttributeError: module 'cv2.aruco' has no attribute 'detectMarkers'`) — it was replaced by a `cv2.aruco.ArucoDetector` class (`detector = cv2.aruco.ArucoDetector(ARUCO_DICTIONARY, cv2.aruco.DetectorParameters())`, then `detector.detectMarkers(frame)`). The dictionary, the returned `corners`/`ids` shapes, and the corner ordering are identical either way — only the call site changes.

---

## :green_square: Step 1: Search State Only

### :red_square: Step 1: Fly and Rotate to Search
- Take off with the `f` key and rotate continuously, `SEARCH_DEG` degrees every `SEARCH_INTERVAL` seconds, with no vision processing at all.
- **Extension beyond the GitHub base**: `single_fly_turnleft`/`turnright` block for as long as the drone takes to complete the turn. Sending one directly inside the main loop every `SEARCH_INTERVAL` would repeatedly stall frame capture and keyboard polling, so it is dispatched through `run_flight()` — a daemon thread guarded by a lock that silently drops a new command if the previous one hasn't finished yet.

#### :o:Practice[armarker_search]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\hula_armarker_search.py`)
- `hula_armarker_search.py`
```python
import sys
import os
import time
import threading

import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture

DRONE_IP        = "192.168.100.XXX"  # set to your drone's address (see drone_control.md Step 0)
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
        print(f"Connecting to drone at {DRONE_IP}...")
        api.connect(DRONE_IP)
        time.sleep(3.0)
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
```

> [!NOTE]
> ### Explanation: armarker_search
> - **`rotate(deg)` sign convention**: `single_fly_turnleft`/`turnright` each take an unsigned angle. `rotate()` folds both into one signed helper so the rest of the program can just say "turn by this many degrees, in this direction" without an `if` at every call site. `SEARCH_DEG = 10` is turned left; `rotate(-SEARCH_DEG)` is what actually spins the search — negative values turn right, positive values turn left.
> - **Why `run_flight()` matters here already**: a single blocking command (like a one-off takeoff) would only freeze the loop briefly. Step 1 needs to *repeat* a rotation command every 1.2 s for as long as the drone searches, so without dispatching it to a daemon thread, the video window and keyboard input would freeze solid for that entire duration — `run_flight()` is required from the very first practice, not just once flight gets more complex.
> - **`last_search` and wall-clock time**: `time.time()` (not the video stream's timestamp) is used to pace the search, because there is no `DetectionTimer` yet at this stage — it's the simplest possible interval check. Step 4 switches the landing timer to the stream clock once `DetectionTimer` is introduced; see that step's note for why.
> - **No `State` enum yet**: with only one state, an `Enum` would just be ceremony. A plain on-screen label (`"State: SEARCH"`) is enough until Step 4 needs to distinguish `SEARCH` / `FOLLOW` / `LAND`.
> - **`_flight_lock.acquire(timeout=3.0)` around the touchdown call**: `_flight_lock.acquire(blocking=False)` inside `run_flight()` only stops *new* rotate commands from queuing up — it does nothing to protect `single_fly_touchdown()`, which is called directly from the main thread without touching the lock at all. Because this program never leaves `SEARCH`, a rotate command is dispatched roughly every `SEARCH_INTERVAL` (1.2 s) for as long as it flies, so there's a real chance a background thread is still inside a blocking `single_fly_turnleft()`/`turnright()` call at the exact moment `q` is pressed. Calling two `pyhula` methods on the same `api` object from two threads at once can hang. A **bounded** `acquire(timeout=3.0)` (rather than an unbounded `with _flight_lock:`) waits for an in-flight rotate to finish first, but gives up after 3 seconds and sends `touchdown` anyway — if the background thread's SDK call itself never returns for some reason, the program must still be able to land and exit rather than hang forever. `acquired` is tracked so the lock is only released in the `finally` block if it was actually acquired (releasing a lock you never acquired raises `RuntimeError`).
> - **`cv2.destroyAllWindows()` is skipped**: on Windows, calling it after the video loop has stopped pumping `cv2.waitKey()` can hang indefinitely waiting on the HighGUI window's message loop — this was confirmed by testing (`cap.release() done.` printed, but nothing after it). Since the process force-exits right after anyway, the OS reclaims the window along with everything else; there's nothing to gain from waiting on a window-close call that may never return.
> - **`os._exit(0)` at the very end**: `pyhula`'s internal video receiver (the thread that decodes the H.264/RTP stream from the drone camera) keeps running once started, and `VideoCapture.release()` in "hula" mode only flips a Python-side `_is_opened` flag — it never tells that thread to stop. If it isn't a daemon thread, `python program.py` can hang indefinitely after your very last `print()` while the interpreter waits at normal shutdown for every thread to join. Since flight-safety-critical work (`touchdown`) is already done by this point, `os._exit(0)` terminates the process immediately rather than waiting.

---

## :green_square: Step 2: Stay State with LED Feedback

### :red_square: Step 2: Detect Marker → Stop and Light the LED
- Extend Step 1: once a marker is detected, stop rotating, hover in place, and light the LED green. If the marker is lost again, resume the exact same search rotation as Step 1 (no special "side search" — this program always falls back to the plain, full rotation).
- **Extension beyond Step 1**: a second lock, `_led_lock`, is added so that an LED command never blocks a pending flight command (or vice versa) — they are independent SDK calls running on independent daemon threads.

#### :o:Practice[armarker_stay]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\hula_armarker_stay.py`)
- `hula_armarker_stay.py`
```python
import sys
import os
import time
import threading

import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture

DRONE_IP        = "192.168.100.XXX"  # set to your drone's address (see drone_control.md Step 0)
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
        print(f"Connecting to drone at {DRONE_IP}...")
        api.connect(DRONE_IP)
        time.sleep(3.0)
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
```

> [!NOTE]
> ### Explanation: armarker_stay
> - **`single_fly_lamplight(r, g, b, time, mode)` parameters**: the SDK signature is `(r, g, b, time, mode)` — **not** `(r, g, b, duration, count)`. `time` is the light's active duration **in seconds**, and `mode` selects the lighting pattern: `1` = solid on, `2` = off, `4` = RGB color-cycle, `16` = rainbow, `32` = blink, `64` = breathing. So `run_led(api.single_fly_lamplight, 0, 255, 0, 2, 32)` means "green, for 2 seconds, blinking" — it is re-issued every time the state changes because the effect only lasts for the `time` given, not indefinitely.
> - **`ids is not None` before `len(ids)`**: `ARUCO_DETECTOR.detectMarkers()` returns `ids=None` (not an empty array) when nothing is found. Checking `len(ids) > 0` first would raise `TypeError: object of type 'NoneType' has no len()`, so the `None` check must always come first.
> - **`is_staying` as a plain `bool`**: with only two states, a full `State(Enum)` would be premature — the same reasoning as Step 1. The formal `Enum` is introduced only in Step 4, once a third state (`LAND`) actually needs distinguishing.
> - **`acquire(timeout=3.0)` on both locks around the touchdown call**: same reasoning as Step 1, extended to the LED lock now that one exists. A background thread can still be inside a blocking `single_fly_turnleft()`/`turnright()` *or* `single_fly_lamplight()` call when `q` is pressed; the bounded acquire waits for whichever is in flight to finish, but only for up to 3 seconds each — long enough for a normal in-flight command, short enough that a genuinely stuck SDK call still lets the program proceed to `touchdown` and exit instead of hanging forever.
> - **`cv2.destroyAllWindows()` is skipped, and `os._exit(0)` at the very end**: same reasoning as Step 1 — `destroyAllWindows()` can hang on Windows once `cv2.waitKey()` has stopped being called, and `pyhula`'s internal video receiver thread keeps running after `VideoCapture.release()`, which only flips a Python-side flag rather than actually stopping it. Force-exiting after `touchdown` has already completed avoids both problems.

---

## :green_square: Step 3: Vision-Only Distance & Position Display

### :red_square: Step 3: Estimate Distance and Position (No Flight)
- The drone stays on the ground for this entire practice — only the camera and marker detection run. This lets you safely calibrate `TARGET_MARKER_PX` and watch the distance/position readout change in real time as you move a printed marker around, with zero flight risk.
- The math introduced here (`marker_metrics()`, dead-band comparisons) is reused unchanged for actual flight control in Step 4.

```
Frame center X = 0.5
Marker center X = cx  (0.0 = left edge, 1.0 = right edge)

x_err = cx - 0.5
  x_err > +0.15  →  marker is right of center  →  "RIGHT"
  x_err < -0.15  →  marker is left of center   →  "LEFT"
  |x_err| ≤ 0.15 →  "CENTER"

diag_px = pixel distance: top-left corner → bottom-right corner
  diag_px > TARGET_MARKER_PX * 1.15  →  "TOO CLOSE"
  diag_px < TARGET_MARKER_PX * 0.85  →  "TOO FAR"
  otherwise                          →  "GOOD"
```

#### :o:Practice[armarker_vision]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\hula_armarker_vision.py`)
- `hula_armarker_vision.py`
```python
import sys
import time

import numpy as np
import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture

DRONE_IP         = "192.168.100.XXX"  # set to your drone's address (see drone_control.md Step 0)
TARGET_MARKER_PX = 150   # desired marker diagonal size (px) -- calibrate per marker size/camera
DEAD_BAND        = 0.15  # tolerance around the target before a category flips

ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

def marker_metrics(corners, idx, frame_w):
    """Return (normalized_center_x, diagonal_size_px) for the marker at corners[idx].

    corners[idx] has shape (1, 4, 2): the marker's 4 corners, ordered
    top-left, top-right, bottom-right, bottom-left. cx is normalized to
    [0, 1] across the frame width. diag_px is the pixel distance between the
    top-left and bottom-right corners -- a rotation-robust proxy for distance
    to the camera (same corner[0]/corner[2] technique as the reference
    AR-marker material).
    """
    pts = corners[idx][0]
    cx = float(pts[:, 0].mean()) / frame_w
    diag_px = float(np.linalg.norm(pts[2] - pts[0]))
    return cx, diag_px

def distance_label(diag_px):
    """Classify the marker's apparent size into a rough distance category."""
    if diag_px > TARGET_MARKER_PX * (1 + DEAD_BAND):
        return "TOO CLOSE", (0, 0, 255)
    if diag_px < TARGET_MARKER_PX * (1 - DEAD_BAND):
        return "TOO FAR", (0, 165, 255)
    return "GOOD", (0, 255, 0)

def position_label(cx):
    """Classify the marker's horizontal position into left/center/right."""
    x_err = cx - 0.5
    if x_err > DEAD_BAND:
        return "RIGHT", (0, 165, 255)
    if x_err < -DEAD_BAND:
        return "LEFT", (0, 165, 255)
    return "CENTER", (0, 255, 0)

def main():
    # 1. Connect first (camera only -- the drone never leaves the ground in this practice)
    try:
        api = pyhula.UserApi()
        print(f"Connecting to drone at {DRONE_IP}...")
        api.connect(DRONE_IP)
        time.sleep(3.0)
    except Exception as e:
        print(f"[ERROR] Failed to setup drone: {e}")
        sys.exit(1)

    # 2. Activate watcher and open stream (no takeoff calls anywhere below)
    with SafeDroneWatcher(api):
        cap = VideoCapture(api)

        print("Video stream active. No flight in this practice -- vision only.")
        print("Press 'q' to quit.")

        while cap.isOpened():
            ret, frame = cap.read()

            key_press = cv2.waitKey(1) & 0xFF
            if key_press == ord('q'):
                break
            if not ret or frame is None:
                continue

            _, w = frame.shape[:2]
            corners, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO_DICTIONARY)
            annotated = frame.copy()

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
                cx, diag_px = marker_metrics(corners, 0, w)

                dist_text, dist_color = distance_label(diag_px)
                pos_text,  pos_color  = position_label(cx)

                cv2.putText(annotated, f"Distance: {dist_text} ({diag_px:.0f}px)",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dist_color, 2)
                cv2.putText(annotated, f"Position: {pos_text} (cx={cx:.2f})",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, pos_color, 2)
            else:
                cv2.putText(annotated, "No marker detected",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 2)

            cv2.imshow("Hula Camera", annotated)

        cap.release()
        cv2.destroyAllWindows()
        print("Resources released successfully.")

if __name__ == "__main__":
    main()
```

> [!NOTE]
> ### Explanation: armarker_vision
> - **Why a no-flight step exists at all**: tuning `TARGET_MARKER_PX` while the drone is airborne means every misjudged threshold causes an unwanted rotation or lurch. This step gives you a safe sandbox for that calibration: print `diag_px` on screen, physically hold the marker at your intended following distance, and adjust the constant until "GOOD" appears — with the drone never leaving the ground.
> - **`marker_metrics()` corner math**: `pts[:, 0].mean()` averages the x-coordinate of all 4 corners to get the marker's center column, then divides by frame width to normalize it to `[0, 1]`. `np.linalg.norm(pts[2] - pts[0])` is the Euclidean distance between corner index 2 (bottom-right) and index 0 (top-left) — the diagonal. Using the diagonal (rather than, say, just the top edge length) keeps the size estimate stable even when the marker is seen at a slight angle or rotated in-plane.
> - **Only `corners[0]` is read**: if more than one marker is visible, `marker_metrics(corners, 0, w)` always looks at the first one OpenCV happened to return. This program does not attempt to pick "the closest" or "a specific ID" — following exactly one target keeps the control logic simple.
> - **Dead-band reuse**: `DEAD_BAND = 0.15` is used for *both* the position and distance checks here. Using one shared tolerance keeps the categories simple and numerically consistent with the Step 4 flight corrections — a threshold you tune here transfers directly.

---

## :green_square: Step 4: Complete Program — State Machine and Marker-Lock Landing

### :red_square: Step 4: Add Follow Control, State Machine, and a Hold-Timer Landing Trigger
- Upgrade Steps 1–3 into the full **`SEARCH` / `FOLLOW` / `LAND`** state machine.
- Landing is triggered by **holding the marker centered and at the target distance continuously for `STAY_MS` (1 second)** — reusing `DetectionTimer` with a grace period so a single noisy detection doesn't reset the hold.

| Item | Step 2 (`hula_armarker_stay.py`) | Step 4 (`test_hula_armarker.py`) |
|---|---|---|
| Marker use | Presence only (found / not found) | Presence **and** `marker_metrics()` (position + distance) |
| State | `bool is_staying` | `State` enum: `SEARCH` / `FOLLOW` / `LAND` |
| Flight correction | None (just hover) | Rotate to re-center, then move forward/back to match `TARGET_MARKER_PX` |
| Landing trigger | None (`q` key only) | Marker locked on (centered + correct distance) for 1 s, or `q` key |

#### :o:Practice[armarker_follow_complete]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\test_hula_armarker.py`)
- `test_hula_armarker.py`
```python
import sys
import os
import time
import threading
from enum import Enum, auto

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
            stay_timer.start_time = None
            stay_timer.lost_time  = None
            stay_timer.is_reached = False

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
```

> [!NOTE]
> ### Explanation: armarker_follow_complete
> - **`DetectionTimer(STAY_MS, STAY_GRACE_MS)` as a lock-hold timer**: `stay_timer.update(is_locked and state == State.FOLLOW, current_msec)` is fed `True` only on frames where the marker is *both* well-centered (`|x_err| ≤ DEAD_BAND`) *and* at the right distance (`|size_err| ≤ DEAD_BAND`) *and* the drone is actively in `FOLLOW`. If the marker drifts out of lock for less than `STAY_GRACE_MS` (300 ms) — e.g. a single noisy detection — the accumulated hold time is preserved; a longer drop resets it.
> - **Why the condition is gated by `state == State.FOLLOW`**: without this, a marker that happens to be centered and at the right distance while still in `SEARCH` (e.g. spotted mid-rotation, before the corrective turn) could start accumulating lock time. Gating on `FOLLOW` guarantees the hold only counts once the drone has actually committed to tracking it.
> - **Order of corrections in `FOLLOW`**: horizontal centering (`x_err`) is corrected before forward/back distance (`size_err`) — the `elif` chain means only one correction is ever sent per frame, and centering always wins first. This prevents the drone from drifting sideways while it is still approaching or backing away from the marker.
> - **`ids is not None and len(ids) > 0`**: repeated from Steps 2–3 — `detectMarkers()` returns `None` for `ids` (not an empty array) when nothing is found, so the `None` check must come first.
> - **`acquire(timeout=3.0)` on both locks around the touchdown call**: same fix as Steps 1–2. `run_flight()`/`run_led()` only use `acquire(blocking=False)` to drop *new* commands when one is already running — they do nothing to protect `single_fly_touchdown()`, which used to be called directly from the main thread with no lock at all. If `q` is pressed while `SEARCH` is still rotating (no marker found yet), a background thread can easily still be mid-call inside `single_fly_turnleft()`/`turnright()`; calling `single_fly_touchdown()` on the same `api` object from the main thread at that exact moment can race with it and hang. A **bounded** wait (3 s per lock) gives an in-flight command a chance to finish normally, but guarantees the program still proceeds to `touchdown` and exits even if a background SDK call never returns for some reason — an unbounded `with` would hang forever in that case. `flight_acquired`/`led_acquired` are tracked so `finally` only releases a lock that was actually acquired.
> - **`cv2.destroyAllWindows()` is skipped, and `os._exit(0)` at the very end**: same reasoning as Steps 1–2 — `destroyAllWindows()` can hang on Windows once `cv2.waitKey()` has stopped being called, and `pyhula`'s internal video receiver thread keeps running after `VideoCapture.release()`, which only flips a Python-side flag rather than actually stopping it. Force-exiting after `touchdown` has already completed avoids both problems.
> - **Operation summary**:

| Action | Drone response |
|---|---|
| Press `f` | Takeoff → begin search rotation |
| Show marker | Stop rotation — enter `FOLLOW` state |
| Move marker left / right | Drone rotates to re-center it |
| Move marker closer / farther | Drone backs up / moves forward |
| Hold marker centered + correct distance for 1 s | LED turns red → land |
| Hide marker | Return to `SEARCH` rotation |
| Press `q` | Force-land immediately |

---

[back to the top page](../README.md)
