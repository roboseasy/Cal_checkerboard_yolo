"""Overlay robot-base (x, y) coordinates on every chessboard grid intersection.

- Camera calibration: calib/camera_params.npz (from 01.calibrate.py)
- Board->robot rigid transform: calib/robot_board.npz (from 03.robot_calib.py)
- Each frame:
    1. find chessboard inner corners (13x9 by default)
    2. solvePnP -> board pose in camera
    3. project the 15x11 outer-grid intersections
    4. compute robot (x, y) at each intersection and draw a label
"""
import os
import sys
import time
import numpy as np
import cv2
import yaml


def load_config():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "config.yaml"), "r") as f:
        return yaml.safe_load(f)


def build_inner_object_points(pattern_size, square_size_mm):
    """13x9 inner corners in mm, origin at the first inner corner.

    This matches what findChessboardCorners returns and is what solvePnP needs.
    """
    cols, rows = pattern_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_mm
    return objp


def build_outer_grid_mm(pattern_size, square_size_mm):
    """15x11 outer-grid corners in board frame B (mm) as 3D points (z=0).

    Outer index (i, j), i in 0..cols, j in 0..rows.
    In the solvePnP coordinate system the first INNER corner is at (0,0),
    so outer (i, j) corresponds to ((i-1)*s, (j-1)*s) in mm.
    Returns: pts (N, 3) float32, ij (N, 2) int.
    """
    cols, rows = pattern_size
    pts = []
    ij = []
    for j in range(rows + 1):
        for i in range(cols + 1):
            pts.append([(i - 1) * square_size_mm,
                        (j - 1) * square_size_mm,
                        0.0])
            ij.append([i, j])
    return np.float32(pts), np.int32(ij)


def main():
    cfg = load_config()
    pattern_size = tuple(cfg["board"]["pattern_size"])
    square_size_mm = float(cfg["board"]["square_size_mm"])
    s_m = square_size_mm * 1e-3
    cam = cfg["camera"]
    camera_file = cfg["calib"]["file"]
    rb_file = cfg["robot_calib"]["file"]
    z_board = float(cfg["robot_calib"]["z_board_m"])

    disp = cfg.get("display", {})
    stride = int(disp.get("label_stride", 1))
    unit = str(disp.get("label_unit", "m"))
    decimals = int(disp.get("label_decimals", 3))
    font_scale = float(disp.get("label_font_scale", 0.35))

    if not os.path.exists(camera_file):
        print(f"Missing {camera_file}. Run 01.calibrate.py first.", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(rb_file):
        print(f"Missing {rb_file}. Run 03.robot_calib.py first.", file=sys.stderr)
        sys.exit(1)

    cam_data = np.load(camera_file)
    K = cam_data["K"]
    dist = cam_data["dist"]
    rb_data = np.load(rb_file)
    R_br = rb_data["R"]              # 2x2: board(m) -> robot(m)
    t_br = rb_data["t"]              # 2,
    theta = float(rb_data["theta"])
    print(f"[ar_grid] camera RMS={float(cam_data['rms']):.3f}px  "
          f"robot_calib theta={np.degrees(theta):+.2f} deg  "
          f"t=[{t_br[0]:+.4f}, {t_br[1]:+.4f}] m")

    objp = build_inner_object_points(pattern_size, square_size_mm)
    grid_pts_mm, ij = build_outer_grid_mm(pattern_size, square_size_mm)

    # Precompute robot (x, y, z) for every outer grid corner
    cols, rows = pattern_size
    robot_xyz = np.zeros((len(grid_pts_mm), 3), dtype=np.float64)
    for k, (i, j) in enumerate(ij):
        b_xy = np.array([i * s_m, j * s_m])
        r_xy = R_br @ b_xy + t_br
        robot_xyz[k] = (r_xy[0], r_xy[1], z_board)

    cap = cv2.VideoCapture(int(cam["index"]))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cam["fourcc"])))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam["width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam["height"]))
    cap.set(cv2.CAP_PROP_FPS, int(cam["fps"]))
    if not cap.isOpened():
        print("Cannot open webcam.", file=sys.stderr)
        sys.exit(1)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                  | cv2.CALIB_CB_NORMALIZE_IMAGE
                  | cv2.CALIB_CB_FAST_CHECK)

    print("[ar_grid] press q to quit")
    last_t = time.time()
    fps = 0.0
    scale_disp = 1.0 if unit == "m" else 1000.0

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
                imgpts, _ = cv2.projectPoints(grid_pts_mm, rvec, tvec, K, dist)
                imgpts = imgpts.reshape(-1, 2)

                for k, (i, j) in enumerate(ij):
                    if (i % stride) or (j % stride):
                        continue
                    u, v = int(round(imgpts[k, 0])), int(round(imgpts[k, 1]))
                    cv2.circle(frame, (u, v), 2, (0, 255, 255), -1, cv2.LINE_AA)
                    rx, ry = robot_xyz[k, 0] * scale_disp, robot_xyz[k, 1] * scale_disp
                    label = f"({rx:.{decimals}f},{ry:.{decimals}f})"
                    cv2.putText(frame, label, (u + 3, v - 3),
                                cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                                (255, 255, 255), 1, cv2.LINE_AA)

        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        hud = (f"{fps:5.1f} FPS  detected={found}  "
               f"unit={unit}  z={z_board:.4f}m  theta={np.degrees(theta):+.1f}deg")
        cv2.putText(frame, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("ar_grid", frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
