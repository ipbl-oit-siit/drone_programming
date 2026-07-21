import sys
import cv2
import numpy as np
import time
import pyhula
from my_libs.my_av2 import VideoCapture
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.detection_timer import DetectionTimer

DRONE_IP = "192.168.100.116"

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

                if area > 30000:
                    cv2.putText(frame, "Target Reached!", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    api.single_fly_touchdown()
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