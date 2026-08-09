"""Show which physical corner carries which board index — run before step 3.

`robot_calib.touch_points` in config.yaml is written as outer-grid indices
(i, j), but nothing else tells you which corner of the printed board is (0, 0).
That ordering comes from findChessboardCorners and depends on how the board
happens to sit in the frame, so it is not something you can assume. Touch the
wrong corner and 03.robot_calib.py still reports a healthy RMS while every
robot coordinate comes out mirrored or rotated by 90 degrees.

This projects the outer grid onto the live camera view, marks the origin, and
draws the +X / +Y directions, so the corners to touch can be picked by eye.

    python 02b.board_index.py

Keys: a = label every grid point instead of just the suggested ones
      s = save a snapshot next to the calibration file
      q = quit
"""
import os
import sys
import time

import numpy as np
import cv2

from config_util import load_config, resolve_path
from frame_source import FrameSourceError, open_frame_source

# The spread suggested in the README: origin plus points that give a baseline
# along both the long and the short side of the board.
SUGGESTED_POINTS = ((0, 0), (1, 1), (1, 4), (13, 1), (13, 4))

SNAPSHOT_NAME = "board_index.png"


def build_inner_object_points(pattern_size, square_size_mm):
    cols, rows = pattern_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_mm
    return objp


def build_outer_grid(pattern_size, square_size_mm):
    """3D outer-grid points in the board frame, plus their (i, j) indices.

    The outer grid sits one square outside the inner corners on every side, so
    outer (1, 1) coincides with the first inner corner.
    """
    cols, rows = pattern_size
    n_i = cols + 2
    n_j = rows + 2
    points = []
    indices = []
    for j in range(n_j):
        for i in range(n_i):
            points.append(((i - 1) * square_size_mm, (j - 1) * square_size_mm, 0.0))
            indices.append((i, j))
    return np.float32(points), indices, n_i, n_j


def put_label(img, text, org, color, scale):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 1, cv2.LINE_AA)


def main():
    cfg = load_config()
    pattern_size = tuple(cfg["board"]["pattern_size"])
    sq_mm = float(cfg["board"]["square_size_mm"])
    color_x = tuple(cfg["colors"]["x"])
    color_y = tuple(cfg["colors"]["y"])
    color_z = tuple(cfg["colors"]["z"])

    disp = cfg.get("display", {})
    stride = max(int(disp.get("label_stride", 1)), 1)
    font_scale = float(disp.get("label_font_scale", 0.35))

    calib_file = resolve_path(cfg["calib"]["file"])
    if not os.path.exists(calib_file):
        print(f"Missing {calib_file}. Run 01.calibrate.py first.", file=sys.stderr)
        sys.exit(1)
    cam_data = np.load(calib_file)
    K, dist = cam_data["K"], cam_data["dist"]
    rms = float(cam_data["rms"])
    print(f"[index] camera RMS={rms:.3f}px")
    if rms >= 1.0:
        print(f"[index] WARNING: RMS {rms:.2f}px is high (want < 1.0). The indices "
              "drawn here stay correct, but every coordinate you derive from this "
              "calibration will be off. Consider re-running 01.calibrate.py.")

    objp = build_inner_object_points(pattern_size, sq_mm)
    grid_3d, grid_ij, n_i, n_j = build_outer_grid(pattern_size, sq_mm)
    print(f"[index] outer grid: i = 0..{n_i - 1}, j = 0..{n_j - 1}")

    # Origin and the two axis tips, drawn as arrows so the directions are obvious.
    axis_len_mm = float(cfg["board"].get("axis_length_mm", 60.0))
    axes_3d = np.float32([
        [-sq_mm, -sq_mm, 0.0],                          # outer (0, 0)
        [-sq_mm + axis_len_mm, -sq_mm, 0.0],            # +X
        [-sq_mm, -sq_mm + axis_len_mm, 0.0],            # +Y
    ])

    try:
        cap = open_frame_source(cfg)
    except FrameSourceError as e:
        print(f"Cannot open the camera.\n{e}", file=sys.stderr)
        sys.exit(1)
    print(f"[index] source: {cap.describe()}")
    print("[index] a=label all  s=snapshot  q=quit")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                  | cv2.CALIB_CB_NORMALIZE_IMAGE
                  | cv2.CALIB_CB_FAST_CHECK)

    label_all = False
    last_t = time.time()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[index] stream dropped.", file=sys.stderr)
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, find_flags)

        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, dist,
                                              flags=cv2.SOLVEPNP_ITERATIVE)
            if ok_pnp:
                pts, _ = cv2.projectPoints(grid_3d, rvec, tvec, K, dist)
                pts = pts.reshape(-1, 2)
                h, w = frame.shape[:2]

                for (i, j), (u, v) in zip(grid_ij, pts):
                    iu, iv = int(round(u)), int(round(v))
                    if not (0 <= iu < w and 0 <= iv < h):
                        continue
                    suggested = (i, j) in SUGGESTED_POINTS
                    if (i, j) == (0, 0):
                        continue          # drawn with the axes below
                    if suggested:
                        cv2.circle(frame, (iu, iv), 6, color_z, 2, cv2.LINE_AA)
                        put_label(frame, f"({i},{j})", (iu + 8, iv - 8),
                                  (255, 255, 255), max(font_scale, 0.45))
                    else:
                        cv2.circle(frame, (iu, iv), 2, (0, 255, 255), -1, cv2.LINE_AA)
                        if label_all and not (i % stride) and not (j % stride):
                            put_label(frame, f"{i},{j}", (iu + 3, iv - 3),
                                      (200, 200, 200), font_scale)

                # Origin marker and axis arrows.
                axes, _ = cv2.projectPoints(axes_3d, rvec, tvec, K, dist)
                o, ax, ay = [tuple(np.int32(p).ravel()) for p in axes]
                cv2.arrowedLine(frame, o, ax, color_x, 3, cv2.LINE_AA, tipLength=0.2)
                cv2.arrowedLine(frame, o, ay, color_y, 3, cv2.LINE_AA, tipLength=0.2)
                put_label(frame, "+X (i)", (ax[0] + 6, ax[1]), color_x, 0.6)
                put_label(frame, "+Y (j)", (ay[0] + 6, ay[1]), color_y, 0.6)
                cv2.circle(frame, o, 9, color_z, -1, cv2.LINE_AA)
                cv2.circle(frame, o, 9, (255, 255, 255), 2, cv2.LINE_AA)
                put_label(frame, "ORIGIN (0,0)", (o[0] + 12, o[1] + 6),
                          (255, 255, 255), 0.6)

        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        hud = (f"{fps:5.1f} FPS  board={found}  "
               f"{'all labels' if label_all else 'suggested points only'}")
        cv2.putText(frame, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("board_index", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('a'):
            label_all = not label_all
        if key == ord('s'):
            out = os.path.join(os.path.dirname(calib_file), SNAPSHOT_NAME)
            cv2.imwrite(out, frame)
            print(f"[index] saved -> {out}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
