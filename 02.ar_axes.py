"""Real-time 3D axes overlay on a chessboard.

Loads K, dist saved by 01.calibrate.py, detects the chessboard each frame,
estimates its pose via solvePnP, and draws X(green) / Y(blue) / Z(red).
"""
import os
import sys
import time
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


def build_box_points(pattern_size, square_size_mm, height_mm):
    """8 corners of a cuboid covering the inner-corner rectangle of the board.

    Order: 4 bottom (z=0) then 4 top (z=-height), each going CCW from origin.
    """
    cols, rows = pattern_size
    w = (cols - 1) * square_size_mm
    h = (rows - 1) * square_size_mm
    return np.float32([
        [0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0],
        [0, 0, -height_mm], [w, 0, -height_mm],
        [w, h, -height_mm], [0, h, -height_mm],
    ])


def draw_cuboid(img, imgpts, color_x, color_y, color_z):
    pts = np.int32(imgpts).reshape(-1, 2)
    bottom = pts[:4]
    top = pts[4:]

    # Translucent top face
    overlay = img.copy()
    cv2.fillPoly(overlay, [top], color_z)
    cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)

    # Bottom rectangle (on the board) — green (X-axis color family)
    cv2.drawContours(img, [bottom], -1, color_x, 2, cv2.LINE_AA)
    # Top rectangle — red (Z color)
    cv2.drawContours(img, [top], -1, color_z, 2, cv2.LINE_AA)
    # Vertical pillars — blue (Y color)
    for i in range(4):
        cv2.line(img, tuple(bottom[i]), tuple(top[i]), color_y, 2, cv2.LINE_AA)


def main():
    cfg = load_config()
    pattern_size = tuple(cfg["board"]["pattern_size"])
    square_size_mm = float(cfg["board"]["square_size_mm"])
    box_height_mm = float(cfg["board"].get(
        "box_height_mm", cfg["board"]["axis_length_mm"]))
    color_x = tuple(cfg["colors"]["x"])
    color_y = tuple(cfg["colors"]["y"])
    color_z = tuple(cfg["colors"]["z"])
    calib_file = resolve_path(cfg["calib"]["file"])

    if not os.path.exists(calib_file):
        print(f"Missing {calib_file}. Run 01.calibrate.py first.", file=sys.stderr)
        sys.exit(1)
    data = np.load(calib_file)
    K = data["K"]
    dist = data["dist"]
    print(f"[ar] loaded calibration (RMS={float(data['rms']):.3f}px)")

    objp = build_object_points(pattern_size, square_size_mm)
    box_3d = build_box_points(pattern_size, square_size_mm, box_height_mm)

    try:
        cap = open_frame_source(cfg)
    except FrameSourceError as e:
        print(f"Cannot open the camera.\n{e}", file=sys.stderr)
        sys.exit(1)
    print(f"[ar] source: {cap.describe()}")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    find_flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )

    print("[ar] press q to quit")
    last_t = time.time()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, find_flags)

        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, dist,
                                              flags=cv2.SOLVEPNP_ITERATIVE)
            if ok_pnp:
                imgpts, _ = cv2.projectPoints(box_3d, rvec, tvec, K, dist)
                draw_cuboid(frame, imgpts, color_x, color_y, color_z)

        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        cv2.putText(frame, f"{fps:5.1f} FPS  detected={found}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("ar_axes", frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
