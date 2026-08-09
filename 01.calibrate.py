"""Camera calibration using a chessboard.

Run this once. Show the printed board to the webcam from many angles/distances
and press SPACE to capture each shot. After target_shots successful captures
the script computes K & dist and saves them to the path in config.yaml.
"""
import os
import sys
import numpy as np
import cv2

from config_util import load_config, resolve_path
from frame_source import FrameSourceError, open_frame_source


def build_object_points(pattern_size, square_size_mm):
    cols, rows = pattern_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_mm
    return objp


def main():
    cfg = load_config()
    pattern_size = tuple(cfg["board"]["pattern_size"])
    square_size_mm = float(cfg["board"]["square_size_mm"])
    calib_file = resolve_path(cfg["calib"]["file"])
    target_shots = int(cfg["calib"]["target_shots"])

    objp = build_object_points(pattern_size, square_size_mm)
    obj_points = []
    img_points = []
    image_size = None

    try:
        cap = open_frame_source(cfg)
    except FrameSourceError as e:
        print(f"Cannot open the camera.\n{e}", file=sys.stderr)
        sys.exit(1)
    print(f"[calibrate] source: {cap.describe()}")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    find_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )

    print("[calibrate] SPACE=capture, r=reset, ESC=quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed", file=sys.stderr)
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, pattern_size, find_flags)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, pattern_size, corners, found)

        hud = f"shots: {len(obj_points)}/{target_shots}  found={found}"
        cv2.putText(display, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2, cv2.LINE_AA)
        help_text = "SPACE=capture  r=reset  ESC=quit"
        h = display.shape[0]
        cv2.putText(display, help_text, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("calibrate", display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("[calibrate] aborted.")
            cap.release()
            cv2.destroyAllWindows()
            return
        if key == ord('r'):
            obj_points.clear()
            img_points.clear()
            print("[calibrate] reset.")
        if key == 32 and found:  # SPACE
            refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp.copy())
            img_points.append(refined)
            print(f"[calibrate] captured {len(obj_points)}/{target_shots}")
            if len(obj_points) >= target_shots:
                break

    cap.release()
    cv2.destroyAllWindows()

    if len(obj_points) < 5:
        print("Not enough captures to calibrate.", file=sys.stderr)
        sys.exit(1)

    print("[calibrate] computing...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None
    )
    print(f"[calibrate] RMS reprojection error: {rms:.4f}px")
    print("K =\n", K)
    print("dist =", dist.ravel())

    os.makedirs(os.path.dirname(calib_file), exist_ok=True)
    np.savez(calib_file, K=K, dist=dist, rms=rms, image_size=np.array(image_size))
    print(f"[calibrate] saved -> {calib_file}")


if __name__ == "__main__":
    main()
