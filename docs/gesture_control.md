# Hand Gesture Recognition and Following Drone Control

[back to the top page](../README.md)

---

## Objectives
- This page extends [drone_control.md](./drone_control.md) with **real-time hand tracking and gesture-based landing**.
- All programs share the same base architecture as the GitHub samples: `VideoCapture` → `while cap.isOpened()` loop → `SafeDroneWatcher` failsafe.
- Two extensions are layered on top of that base: **MediaPipe gesture recognition** and **non-blocking flight commands via threading**.

## Prerequisite
- All practices in [drone_control.md](./drone_control.md) have been completed, especially `hover_and_failsafe_test` and `camera_angle_control`.
- `MediapipeHandLandmark.py` and `MediapipeHandGestureRecognition.py` are placed under `C:\oit\home\ipbl`.
- **Connection note**: Call `api.connect()` with **no argument**. Passing the drone IP directly causes a socket bind error on this environment.

---

## :green_square: Architecture: Extending the GitHub Base Programs

Every practice in [drone_control.md](./drone_control.md) follows this skeleton:

```python
with SafeDroneWatcher(api):
    cap = VideoCapture(api)          # handles RTP init internally
    some_timer = DetectionTimer(...)

    while cap.isOpened():
        ret, frame = cap.read()
        key_press = cv2.waitKey(1) & 0xFF   # ONE waitKey per loop pass
        if key_press == ord('q'):
            break
        if not ret or frame is None:
            continue
        current_msec = cap.get(cv2.CAP_PROP_POS_MSEC)
        ...
```

This page adds two layers on top:

| Layer | What it adds | First introduced |
|---|---|---|
| **MediapipeHandGestureRecognition** | `hand.detect(frame)` → `hand.get_gesture(i)` inside the loop | Step 1 |
| **threading** | `run_flight()` for non-blocking flight commands | Step 3 |

---

## :green_square: MediapipeHandGestureRecognition

`MediapipeHandGestureRecognition` is a **subclass of `MediapipeHandLandmark`**.  
It adds gesture classification on top of all existing landmark detection methods — no existing calls need to change.

| Method | Description |
|---|---|
| `get_gesture(id_hand)` | Returns the gesture name string (e.g. `"Closed_Fist"`) |
| `get_score_gesture(id_hand)` | Returns the confidence score (0.0–1.0) for the gesture |
| `get_landmark(id_hand, id_lm)` | Pixel coordinates — **inherited** |
| `get_normalized_landmark(id_hand, id_lm)` | Normalised coordinates — **inherited** |
| `get_handedness(id_hand)` | `'Right'` or `'Left'` — **inherited** |
| `visualize(frame)` | Landmark overlay image — **inherited** |
| `release()` | Close recognizer — overrides parent |

### Recognizable Gestures

| `get_gesture()` return value | Gesture |
|---|---|
| `"None"` | No recognizable gesture |
| `"Closed_Fist"` | Closed fist（グー） |
| `"Open_Palm"` | Open palm（パー） |
| `"Pointing_Up"` | Index finger pointing up（人差し指UP） |
| `"Thumb_Down"` | Thumbs down |
| `"Thumb_Up"` | Thumbs up |
| `"Victory"` | V sign / Peace sign（チョキ） |
| `"ILoveYou"` | ILY hand sign |

---

## :green_square: Ground Test: Gesture Recognition without Flight

### :red_square: Step 1: Verify Gesture Recognition with the Drone Camera (No Takeoff)
- Confirm that gesture recognition works with the drone's live video stream before any flight attempt.
- The drone stays on the ground. `SafeDroneWatcher` and `VideoCapture` are used exactly as in the GitHub base programs.

#### :o:Practice[ground_gesture_test]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\hula_hand_detection_test.py`)
- `hula_hand_detection_test.py`
```python
import sys
import time
import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture
from MediapipeHandGestureRecognition import MediapipeHandGestureRecognition

def main():
    # 1. Connect first
    try:
        api = pyhula.UserApi()
        print("Connecting to drone...")
        api.connect()   # No argument: auto-detects PC Wi-Fi IP
        time.sleep(1.0)
    except Exception as e:
        print(f"[ERROR] Failed to setup drone: {e}")
        sys.exit(1)

    # 2. Activate watcher and open stream
    with SafeDroneWatcher(api):
        cap = VideoCapture(api)   # VideoCapture handles RTP init internally

        hand = MediapipeHandGestureRecognition(
            num_hands=2,
            min_hand_detection_confidence=0.6,
            mode="video",
        )

        print("Stream active. Show your hand to the drone camera.")
        print("Try: Closed_Fist / Open_Palm / Victory / Pointing_Up / Thumb_Up")
        print("Press 'q' to quit.")

        while cap.isOpened():
            ret, frame = cap.read()

            key_press = cv2.waitKey(1) & 0xFF
            if key_press == ord('q'):
                break
            if not ret or frame is None:
                continue

            hand.detect(frame)
            annotated = hand.visualize(frame)

            if hand.num_detected_hands > 0:
                gesture    = hand.get_gesture(0)
                handedness = hand.get_handedness(0)
                score      = hand.get_score_gesture(0)
                print(f"Hand: {handedness}  Gesture: {gesture}  Score: {score:.2f}")
                cv2.putText(annotated, f"{gesture} ({score:.2f})",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            else:
                cv2.putText(annotated, "No hand detected",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 2)

            cv2.imshow("Ground Gesture Test", annotated)

        hand.release()
        cap.release()
        cv2.destroyAllWindows()
        print("Resources released successfully.")

if __name__ == "__main__":
    main()
```

> [!NOTE]
> ### Explanation: ground_gesture_test
> - **`VideoCapture(api)`**: Calling `VideoCapture` with the `pyhula.UserApi` instance activates Hula mode. The constructor internally calls `Plane_cmd_swith_rtp(0)` and `single_fly_flip_rtp()`, then waits up to 5 seconds for the first frame. If `cap.isOpened()` returns `False` immediately, the H.264 stream did not initialise in time — power-cycle the drone and retry.
> - **`get_gesture(0)` return value**: Returns a string such as `"Closed_Fist"`. If no hand is detected (`num_detected_hands == 0`), never call `get_gesture()` — it prints `'no hand'` and returns `None`. Always guard with `if hand.num_detected_hands > 0` first.
> - **Model file**: `MediapipeHandGestureRecognition` uses `gesture_recognizer.task` (different from `hand_landmarker.task` used by the parent class). It is downloaded automatically to `learned_models/mediapipe/` on first use.
> - **`mode="video"`**: Enables temporal tracking across frames, significantly improving gesture stability compared to `mode="image"`. Always use `"video"` mode inside a real-time loop.

---

## :green_square: Airborne Hand Tracking

### :red_square: Step 2: Hover and Monitor Hands in Flight
- Take off with the `f` key (same pattern as `hula_hover_test.py`) and display detected gestures during a timed hover.
- This step confirms that gesture detection continues to work while the drone vibrates in flight.

#### :o:Practice[hover_gesture_detection]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\hula_hand_hover.py`)
- `hula_hand_hover.py`
```python
import sys
import time
import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture
from my_libs.detection_timer import DetectionTimer
from MediapipeHandGestureRecognition import MediapipeHandGestureRecognition

HOVER_MS   = 10000.0   # Hover duration [ms]
CRUISE_CM  = 50        # Ascent after takeoff [cm]

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
        hand = MediapipeHandGestureRecognition(
            num_hands=1,
            min_hand_detection_confidence=0.6,
            mode="video",
        )

        hover_timer = DetectionTimer(target_ms=HOVER_MS)
        is_airborne = False

        print("Video stream active.")
        print(">>> TO TAKE OFF  : Press 'f' inside the video window <<<")
        print(">>> TO INTERRUPT : Press 'q' or [Ctrl+C] <<<")

        while cap.isOpened():
            ret, frame = cap.read()

            key_press = cv2.waitKey(1) & 0xFF
            if key_press == ord('q'):
                print("[INTERRUPT] 'q' pressed.")
                break
            if not ret or frame is None:
                continue

            current_msec = cap.get(cv2.CAP_PROP_POS_MSEC)

            # --- Takeoff on 'f' key ---
            if not is_airborne:
                if key_press == ord('f'):
                    print("--- Starting Takeoff ---")
                    api.single_fly_takeoff()
                    api.single_fly_up(CRUISE_CM)
                    hover_timer.start_time = cap.get(cv2.CAP_PROP_POS_MSEC)
                    is_airborne = True
                    print(f"Airborne. Hovering for {HOVER_MS/1000:.0f}s.")
                    continue
                else:
                    cv2.putText(frame, "STANDBY ON GROUND | Press 'f' to Takeoff",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imshow("Hand Hover Test", frame)
                    continue

            # --- Timed hover ---
            if hover_timer.update(True, current_msec):
                print(f"[SUCCESS] {HOVER_MS/1000:.0f}-second hover complete.")
                break

            # --- Gesture detection during hover ---
            hand.detect(frame)
            annotated = hand.visualize(frame)

            gesture = hand.get_gesture(0) if hand.num_detected_hands > 0 else "None"
            elapsed = current_msec - hover_timer.start_time
            cv2.putText(annotated,
                        f"HOVERING | {elapsed/1000:.1f} / {HOVER_MS/1000:.0f}s",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated, f"Gesture: {gesture}",
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.imshow("Hand Hover Test", annotated)

        print("Sending safe touchdown command...")
        api.single_fly_touchdown()

        hand.release()
        cap.release()
        cv2.destroyAllWindows()
        print("Resources released successfully.")

if __name__ == "__main__":
    main()
```

> [!NOTE]
> ### Explanation: hover_gesture_detection
> - **`hover_timer.start_time = cap.get(cv2.CAP_PROP_POS_MSEC)`**: The hover timer baseline is set immediately after `single_fly_up()` completes, using the stream's wall-clock milliseconds — the same technique as `hula_hover_test.py`. Setting it after the blocking takeoff calls ensures the 10-second countdown begins from the moment the drone is actually stable and airborne.
> - **`api.single_fly_barrier_aircraft(False)`**: Disables the proximity sensor that can cause unexpected avoidance manoeuvres. Always call this before any flight sequence in an open indoor space.
> - **Takeoff is blocking**: `single_fly_takeoff()` and `single_fly_up()` both block the Python thread until the drone reports completion. After the blocking calls, a `continue` is issued to re-enter the loop and fetch a fresh `current_msec` — the same pattern used in `hula_vision_control.py`.

---

## :green_square: Hand Position Following

### :red_square: Step 3: Rotate and Move to Track the Hand
- Extend Step 2 by sending flight commands based on the hand's position in the frame.
- **Extension beyond GitHub base**: `pyhula` commands are blocking. To keep the video loop running at full frame rate while a command executes, each command is dispatched to a daemon thread via `run_flight()`. A `threading.Lock` ensures only one flight command runs at a time.

The following position control logic is used:

```
Frame centre X = 0.5
Hand centre X  = cx  (0.0 = left edge, 1.0 = right edge)

x_err = cx - 0.5
  x_err > +0.15  →  hand is right of centre  →  rotate right
  x_err < -0.15  →  hand is left of centre   →  rotate left
  |x_err| ≤ 0.15 →  dead band — no command

palm_px = pixel distance: wrist → middle-finger MCP
  palm_px > TARGET  →  hand too close  →  move back
  palm_px < TARGET  →  hand too far    →  move forward
  |size_err| ≤ 0.15 →  dead band — no command
```

#### :o:Practice[hand_position_follow]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\hula_hand_follow.py`)
- `hula_hand_follow.py`
```python
import sys
import time
import threading
import numpy as np
import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture
from MediapipeHandGestureRecognition import MediapipeHandGestureRecognition

CRUISE_CM       = 50
TARGET_PALM_PX  = 130     # wrist→MCP9 pixel distance target (calibrate per environment)
MOVE_STEP       = 10      # max cm per command
SEARCH_DEG      = 10      # rotation per search step [deg]
SEARCH_INTERVAL = 1.2     # seconds between search rotations
DEAD_BAND       = 0.15    # ±15% dead zone
CMD_INTERVAL    = 0.25    # min seconds between flight commands

# ── Non-blocking flight command (Extension) ────────────────────────────────
_flight_lock = threading.Lock()

def run_flight(fn, *args):
    """ Dispatch a pyhula command to a daemon thread; skip if one is running. """
    def _t():
        if not _flight_lock.acquire(blocking=False):
            return
        try:
            fn(*args)
        finally:
            _flight_lock.release()
    threading.Thread(target=_t, daemon=True).start()

# ── Utilities ──────────────────────────────────────────────────────────────
def palm_metrics(hand, i, frame_w):
    lm_w = hand.get_landmark(i, MediapipeHandGestureRecognition.WRIST)
    lm_m = hand.get_landmark(i, MediapipeHandGestureRecognition.MIDDLE_FINGER_MCP)
    if lm_w is None or lm_m is None:
        return None, None
    cx      = (int(lm_w[0]) + int(lm_m[0])) / 2.0 / frame_w
    palm_px = float(np.hypot(int(lm_m[0]) - int(lm_w[0]),
                                int(lm_m[1]) - int(lm_w[1])))
    return cx, palm_px

def clamp_int(v, lo, hi):
    return int(max(lo, min(hi, v)))

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
        hand = MediapipeHandGestureRecognition(
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            mode="video",
        )

        is_airborne   = False
        is_following  = False
        last_search   = 0.0
        last_cmd_time = 0.0

        def send_cmd(fn, *args):
            nonlocal last_cmd_time
            if time.time() - last_cmd_time < CMD_INTERVAL:
                return
            last_cmd_time = time.time()
            run_flight(fn, *args)

        def rotate(deg):
            if deg < 0:
                api.single_fly_turnright(-deg)
            else:
                api.single_fly_turnleft(deg)

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
                    print("[SEARCH] Searching for hand...")
                    continue
                else:
                    cv2.putText(frame, "STANDBY ON GROUND | Press 'f' to Takeoff",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imshow("Hand Follow", frame)
                    continue

            # --- Hand detection and following ---
            now = time.time()
            _, w = frame.shape[:2]
            hand.detect(frame)

            if hand.num_detected_hands > 0:
                if not is_following:
                    is_following = True
                    run_flight(api.single_fly_hover_flight, 0.5)
                    print("[FOLLOW] Hand acquired.")

                cx, palm_px = palm_metrics(hand, 0, w)
                if cx is not None and palm_px is not None:
                    x_err    = cx - 0.5
                    size_err = (palm_px - TARGET_PALM_PX) / TARGET_PALM_PX

                    if x_err > DEAD_BAND:
                        send_cmd(rotate, -clamp_int(x_err * 40, 1, MOVE_STEP))
                    elif x_err < -DEAD_BAND:
                        send_cmd(rotate,  clamp_int(-x_err * 40, 1, MOVE_STEP))
                    elif abs(size_err) > DEAD_BAND:
                        if size_err > 0:
                            send_cmd(api.single_fly_back,
                                        clamp_int(size_err * 40, 1, MOVE_STEP))
                        else:
                            send_cmd(api.single_fly_forward,
                                        clamp_int(-size_err * 40, 1, MOVE_STEP))
            else:
                if is_following:
                    is_following = False
                    print("[SEARCH] Hand lost — resuming search.")
                if now - last_search >= SEARCH_INTERVAL:
                    send_cmd(rotate, -SEARCH_DEG)
                    last_search = now

            annotated   = hand.visualize(frame)
            state_label = "FOLLOW" if is_following else "SEARCH"
            state_color = (0, 255, 0) if is_following else (0, 200, 255)
            cv2.putText(annotated, f"State: {state_label}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)
            cv2.imshow("Hand Follow", annotated)

        print("Sending safe touchdown command...")
        api.single_fly_touchdown()

        hand.release()
        cap.release()
        cv2.destroyAllWindows()
        print("Resources released successfully.")

if __name__ == "__main__":
    main()
```

> [!NOTE]
> ### Explanation: hand_position_follow
> - **`run_flight(fn, *args)` — the key extension**: All `pyhula` flight commands block until the drone reports completion. Without threading, `single_fly_turnright(10)` would freeze the video loop for ~0.5 s per call, causing the window to hang and `cap.read()` to buffer frames. By running the command in a daemon thread, the loop continues uninterrupted. The `Lock.acquire(blocking=False)` pattern ensures that if a command is still executing, the next incoming command is simply discarded rather than queued.
> - **`CMD_INTERVAL = 0.25`**: Even with non-blocking execution, sending too many commands overwhelms the drone's UDP receiver. This per-command cooldown keeps the rate at ≤ 4 Hz.
> - **`palm_metrics()`**: Uses the pixel distance between `WRIST` (landmark 0) and `MIDDLE_FINGER_MCP` (landmark 9) as a proxy for hand-to-camera distance. `TARGET_PALM_PX = 130` is a calibration starting point; adjust by printing `palm_px` at your working distance.
> - **Dead band `DEAD_BAND = 0.15`**: Errors smaller than ±15% are ignored. This prevents the drone from oscillating when the hand is approximately centred.

---

## :green_square: Complete Program: State Machine and Gesture Landing

### :red_square: Step 4: Add Gesture Landing, State Machine, and LED Feedback
- Upgrade Step 3 by adding a full **SEARCH / FOLLOW / LAND** state machine, a gesture-based landing trigger, and LED status feedback.
- `DetectionTimer` requires `Closed_Fist` to be held for **1 second** before landing, using the same chattering-prevention mechanism as the gimbal control in `hula_vision_control.py`.
- A **separate LED lock** (`_led_lock`) prevents LED commands from blocking the flight command thread.

| Item | Step 3 (`hula_hand_follow.py`) | Step 4 (`test_hula_hands.py`) |
|---|---|---|
| Fist detection | None | `get_gesture(0) == "Closed_Fist"` via `DetectionTimer` |
| State | `bool is_following` | `State` enum: SEARCH / FOLLOW / LAND |
| LED | None | `run_led()` with separate lock |
| Landing trigger | `q` key only | `Closed_Fist` (1 s) or `q` key |

#### :o:Practice[hand_follow_complete]
- Save the following sample code as a python file, and execute it. (`C:\oit\home\ipbl\test_hula_hands.py`)
- `test_hula_hands.py`
```python
import sys
import time
import threading
from enum import Enum, auto

import numpy as np
import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture
from my_libs.detection_timer import DetectionTimer
from MediapipeHandGestureRecognition import MediapipeHandGestureRecognition

# ── Tuning parameters ──────────────────────────────────────────────────────
CRUISE_CM       = 50
TARGET_PALM_PX  = 130
MOVE_STEP       = 10
SEARCH_DEG      = 10
SEARCH_INTERVAL = 1.2
FIST_MS         = 1000.0
FIST_GRACE_MS   = 300.0
DEAD_BAND       = 0.15
CMD_INTERVAL    = 0.25
MHL = MediapipeHandGestureRecognition

_HAND_CONNECTIONS = [
    (0, 1),  (1, 2),  (2, 3),  (3, 4),
    (0, 5),  (5, 6),  (6, 7),  (7, 8),
    (5, 9),  (9, 10), (10,11), (11,12),
    (9, 13), (13,14), (14,15), (15,16),
    (0, 17), (13,17), (17,18), (18,19), (19,20),
]

class State(Enum):
    SEARCH = auto()
    FOLLOW = auto()
    LAND   = auto()

# ── Non-blocking execution ─────────────────────────────────────────────────
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

# ── Utilities ──────────────────────────────────────────────────────────────
def palm_metrics(hand, i, frame_w):
    lm_w = hand.get_landmark(i, MHL.WRIST)
    lm_m = hand.get_landmark(i, MHL.MIDDLE_FINGER_MCP)
    if lm_w is None or lm_m is None:
        return None, None
    cx      = (int(lm_w[0]) + int(lm_m[0])) / 2.0 / frame_w
    palm_px = float(np.hypot(int(lm_m[0]) - int(lm_w[0]),
                                int(lm_m[1]) - int(lm_w[1])))
    return cx, palm_px

def clamp_int(v, lo, hi):
    return int(max(lo, min(hi, v)))

def draw_hands(image, hand, target_idx):
    out = image.copy()
    for i in range(hand.num_detected_hands):
        color = (0, 255, 0) if i == target_idx else (200, 100, 0)
        pts = [None] * 21
        for j in range(21):
            lm = hand.get_landmark(i, j)
            if lm is not None:
                pts[j] = (int(lm[0]), int(lm[1]))
        for a, b in _HAND_CONNECTIONS:
            if pts[a] and pts[b]:
                cv2.line(out, pts[a], pts[b], color, 2, cv2.LINE_AA)
        for pt in pts:
            if pt:
                cv2.circle(out, pt, 5, color, -1, cv2.LINE_AA)
        label = hand.get_handedness(i) or ""
        if pts[0]:
            cv2.putText(out, label, (pts[0][0] + 8, pts[0][1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    return out

# ── Main ───────────────────────────────────────────────────────────────────
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
        hand = MHL(
            num_hands                     = 2,
            min_hand_detection_confidence = 0.6,
            min_hand_presence_confidence  = 0.5,
            min_tracking_confidence       = 0.5,
            mode                          = "video",
        )

        state         = State.SEARCH
        fist_timer    = DetectionTimer(FIST_MS, FIST_GRACE_MS)
        is_airborne   = False
        last_search   = 0.0
        last_cmd_time = 0.0
        prev_led      = -1

        def send_cmd(fn, *args):
            nonlocal last_cmd_time
            if time.time() - last_cmd_time < CMD_INTERVAL:
                return
            last_cmd_time = time.time()
            run_flight(fn, *args)

        def rotate(deg):
            if deg < 0:
                api.single_fly_turnright(-deg)
            else:
                api.single_fly_turnleft(deg)

        def reset_fist():
            fist_timer.start_time = None
            fist_timer.lost_time  = None
            fist_timer.is_reached = False

        print("Video stream active.")
        print(">>> TO TAKE OFF  : Press 'f' inside the video window <<<")
        print(">>> TO QUIT      : Press 'q' or [Ctrl+C] <<<")
        print("  Show hand       → Follow")
        print("  Closed_Fist 1s  → Land")

        while cap.isOpened():
            ret, frame = cap.read()

            key_press = cv2.waitKey(1) & 0xFF
            if key_press == ord('q'):
                print("[INTERRUPT] 'q' pressed.")
                break
            if not ret or frame is None:
                continue

            current_msec = cap.get(cv2.CAP_PROP_POS_MSEC)

            # --- Takeoff on 'f' key ---
            if not is_airborne:
                if key_press == ord('f'):
                    print("--- Starting Takeoff ---")
                    api.single_fly_takeoff()
                    api.single_fly_up(CRUISE_CM)
                    is_airborne   = True
                    last_search   = time.time()
                    last_cmd_time = 0.0
                    run_flight(rotate, -SEARCH_DEG)
                    print("[SEARCH] Airborne. Searching for hand...")
                    continue
                else:
                    cv2.putText(frame, "STANDBY ON GROUND | Press 'f' to Takeoff",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imshow("Hula Camera", frame)
                    continue

            # --- Hand detection ---
            now = time.time()
            _, w = frame.shape[:2]
            hand.detect(frame)

            target_idx = 0 if hand.num_detected_hands > 0 else None
            right_fist = (
                target_idx is not None and
                hand.get_gesture(0) == "Closed_Fist"
            )

            # --- LED feedback ---
            if target_idx is None:
                led = 0
            elif right_fist and state == State.FOLLOW:
                led = 2
            else:
                led = 1

            if led != prev_led:
                if led == 0:
                    run_led(api.single_fly_lamplight, 0,   0,   0, 1,  2)
                elif led == 1:
                    run_led(api.single_fly_lamplight, 0, 255,   0, 2, 32)
                else:
                    run_led(api.single_fly_lamplight, 255,  0,  0, 2, 32)
                prev_led = led

            # --- Fist timer (stream-clock driven, same as hula_vision_control.py) ---
            fist_reached = fist_timer.update(
                right_fist and state == State.FOLLOW, current_msec)

            fist_prog_s = 0.0
            if fist_timer.start_time is not None and not fist_timer.is_reached:
                fist_prog_s = min(current_msec - fist_timer.start_time,
                                    FIST_MS) / 1000.0

            # --- State machine ---
            if state == State.SEARCH:
                if target_idx is not None:
                    run_flight(api.single_fly_hover_flight, 0.5)
                    cx, _ = palm_metrics(hand, target_idx, w)
                    if cx is not None:
                        x_err = cx - 0.5
                        if x_err > DEAD_BAND:
                            run_flight(rotate,
                                        -clamp_int(x_err * 30, 1, SEARCH_DEG))
                        elif x_err < -DEAD_BAND:
                            run_flight(rotate,
                                        clamp_int(-x_err * 30, 1, SEARCH_DEG))
                    state = State.FOLLOW
                    reset_fist()
                    print("[FOLLOW] Hand acquired — following.")

                elif now - last_search >= SEARCH_INTERVAL:
                    send_cmd(rotate, -SEARCH_DEG)
                    last_search = now

            elif state == State.FOLLOW:
                if fist_reached:
                    print("[LAND] Closed_Fist confirmed — landing.")
                    state = State.LAND
                    break

                if target_idx is None:
                    state = State.SEARCH
                    print("[SEARCH] Hand lost — resuming search.")
                    send_cmd(rotate, -SEARCH_DEG)
                    last_search = now
                    reset_fist()
                else:
                    cx, palm_px = palm_metrics(hand, target_idx, w)
                    if cx is not None and palm_px is not None:
                        x_err    = cx - 0.5
                        size_err = (palm_px - TARGET_PALM_PX) / TARGET_PALM_PX

                        if x_err > DEAD_BAND:
                            send_cmd(rotate,
                                        -clamp_int(x_err * 40, 1, MOVE_STEP))
                        elif x_err < -DEAD_BAND:
                            send_cmd(rotate,
                                        clamp_int(-x_err * 40, 1, MOVE_STEP))
                        elif abs(size_err) > DEAD_BAND:
                            if size_err > 0:
                                send_cmd(api.single_fly_back,
                                            clamp_int(size_err * 40, 1, MOVE_STEP))
                            else:
                                send_cmd(api.single_fly_forward,
                                            clamp_int(-size_err * 40, 1, MOVE_STEP))

            # --- Render overlay ---
            annotated = draw_hands(frame, hand, target_idx)
            lc = {State.SEARCH: (0, 200, 255),
                    State.FOLLOW: (0, 255,   0),
                    State.LAND:   (0,   0, 255)}.get(state, (255, 255, 255))
            cv2.putText(annotated, f"State: {state.name}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, lc, 2)
            if fist_prog_s > 0:
                cv2.putText(annotated,
                            f"Fist: {fist_prog_s:.1f}/{FIST_MS/1000:.1f}s",
                            (20, 70), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 100, 255), 2)
            cv2.imshow("Hula Camera", annotated)

        # --- Cleanup ---
        try:
            api.single_fly_lamplight(0, 0, 0, 1, 2)
        except Exception:
            pass
        print("Sending safe touchdown command...")
        api.single_fly_touchdown()

        hand.release()
        cap.release()
        cv2.destroyAllWindows()
        print("Resources released successfully.")

if __name__ == "__main__":
    main()
```

> [!NOTE]
> ### Explanation: hand_follow_complete
> - **`DetectionTimer` with `cap.get(cv2.CAP_PROP_POS_MSEC)`**: The fist landing timer uses the stream clock as its time source — exactly the same approach as `hula_vision_control.py`. This keeps the timer consistent with all other GitHub base programs.
> - **`get_gesture(0) == "Closed_Fist"`**: Delegates fist detection to a dedicated neural network inside MediaPipe. Because `MediapipeHandGestureRecognition` inherits all methods from `MediapipeHandLandmark`, `get_landmark()`, `get_handedness()`, and `visualize()` continue to work without any changes.
> - **Separate `_led_lock`**: LED commands (`single_fly_lamplight`) take ~0.5 s to complete. A dedicated lock prevents an LED call from occupying `_flight_lock` and blocking the next position-control command.
> - **Operation summary**:

| Action | Drone response |
|---|---|
| Press `f` | Takeoff → begin search rotation |
| Show hand | Stop rotation — enter FOLLOW state |
| Move hand left / right | Drone rotates to re-centre hand |
| Move hand closer / farther | Drone backs up / moves forward |
| `Closed_Fist` for 1 s | LED turns red → land |
| Hide hand | Return to SEARCH rotation |
| Press `q` | Force-land immediately |

---

[back to the top page](../README.md)
