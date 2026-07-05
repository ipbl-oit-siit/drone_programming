import sys
import time

sys.path.insert(0, r"C:\oit\home\ipbl")  # 共有の my_libs パッケージをインポートできるようにする

import numpy as np
import cv2
import pyhula
from my_libs.safe_drone_watcher import SafeDroneWatcher
from my_libs.my_av2 import VideoCapture

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
        print("Connecting to drone...")
        api.connect()
        time.sleep(1.0)
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