## :green_square: Step 1: Pink Color Detection and Tracking Control

### :red_square: Step 1: Detect Pink Target → Follow and Touchdown

* **Objective**: Instead of an ArUco marker, this practice uses **HSV color space detection** to identify a **pink object (color target)** and automatically follow, move toward, and land on/near it based on its relative position ($X$-axis offset) and contour area.


* **Flow of Execution**:
1. After takeoff, convert frames to HSV and extract the pink region using `inRange()`.


2. Find the largest contour and compute its area (`area`) and horizontal center point (`target_x`).


3. Use `DetectionTimer` to confirm stable detection over time before initiating control movements.


4. Adjust flight direction left/right based on horizontal offset (`dx`), and trigger an automatic touchdown (`touchdown`) when close enough (`area > 50000`).





| Condition | State / Action | Overlay Text |
| --- | --- | --- |
| `area > 50000` | Target reached (sufficiently close) → **Touchdown** | `Target Reached!`<br> |
| `dx > 60` | Target is to the right → **Fly Right (`single_fly_right`)** | `Moving Right...`<br> |
| `dx < -60` | Target is to the left → **Fly Left (`single_fly_left`)** | `Moving Left...`<br> |
| Otherwise (centered) | Clear path ahead → **Fly Forward (`single_fly_forward`)** | `Moving Forward...`<br> |
| Not detected | Lost target / debouncing → **Searching** | `Searching Pink...`<br> |

#### :o:Practice[pink_tracking]

* Save the following sample code as a Python file (e.g., `C:\oit\home\ipbl\hula_pink_tracking.py`) and execute it.


* `hula_pink_tracking.py`

```python
import sys
import cv2
import numpy as np
import time
import pyhula
from my_libs.my_av2 import VideoCapture
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.detection_timer import DetectionTimer

DRONE_IP = "192.168.100.XXX"

def main():
    try:
        api = pyhula.UserApi()
        print("Connecting to drone at ", DRONE_IP, "...")
        api.connect(DRONE_IP)
        time.sleep(3.0)
    except Exception as e:
        print(f"[ERROR] Failed to setup drone: {e}")
        sys.exit(1)

    dt_color = DetectionTimer(300)

    with SafeDroneWatcher(api):
        cap = VideoCapture(api)
        ht = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        wt = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        center_x = wt // 2

        print("Takeoff...")
        api.single_fly_takeoff()
        time.sleep(3.0)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Pink color HSV range setup (Hue: 160 ~ 180)
            lower_pink = np.array([160, 100, 90])
            upper_pink = np.array([180, 255, 255])
            pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)
            cv2.imshow("pink", pink_mask)

            contours = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            pink_is_detected = False

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                x, y, w, h = cv2.boundingRect(largest_contour)

                if area > 150:
                    pink_is_detected = True
                    target_x = x + (w // 2)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), [0,255,0], 1)
                else:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), [255,255,255], 1)

            if dt_color.update(pink_is_detected, timestamp):
                dx = target_x - center_x

                if area > 50000:
                    cv2.putText(frame, "Target Reached!", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    api.single_fly_touchdown()
                    break
                else:
                    if dx > 60:
                        cv2.putText(frame, "Moving Right...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        api.single_fly_right(distance=20, speed=50)
                    elif dx < -60:
                        cv2.putText(frame, "Moving Left...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        api.single_fly_left(distance=20, speed=50)
                    else:
                        cv2.putText(frame, "Moving Forward...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        api.single_fly_forward(distance=30, speed=50)
            else:
                cv2.putText(frame, "Searching Pink...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("hula", frame)

            if cv2.waitKey(1) == ord('q'):
                api.single_fly_touchdown()
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

```

> [!NOTE]
> ### Explanation: pink_tracking
> 
> 
> * **Extraction via HSV Color Space**: To minimize sensitivity to changing light conditions, the frame is converted from BGR to HSV (`cv2.cvtColor`), and pink pixels are thresholded using a binary mask (`inRange`) across the hue range (Hue: roughly 160–180). You may need to tune `lower_pink` and `upper_pink` depending on your room's ambient lighting.
> 
> 
> * **Noise Filtering**: An area threshold of `area > 150` prevents small background noise artifacts from triggering target locks.
> 
> 
> * **Blocking Movement Commands**: Note that movement calls like `single_fly_right` or `single_fly_forward` are blocking SDK functions in this simplified script. Frame updates pause during execution, which is why movement Step sizes (`distance=20` or `30`) are kept intentionally small.
> 
> 
> 
>
