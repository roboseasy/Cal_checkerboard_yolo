"""Top-down rectified view of the chessboard with robot-base coord labels.

Each frame:
  1. detect chessboard inner corners + solvePnP
  2. project the four OUTER corners of the board into the image
  3. warp that quad to a fixed-size canvas (board only)
  4. draw a marker + (x, y) robot-base label at every outer-grid intersection

Pattern size (cols, rows) refers to INNER corners.
Number of squares     = (cols + 1, rows + 1)
Number of OUTER inter-
sections (grid corners) = (cols + 2, rows + 2)
Board outer span (mm) = ((cols + 1) * sq, (rows + 1) * sq)
"""
import os
import sys
import time
import numpy as np
import cv2
import yaml

PIXELS_PER_MM = 6  # rectified canvas resolution (6 -> 1680x1200)


def load_config():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "config.yaml"), "r") as f:
        return yaml.safe_load(f)


def build_inner_object_points(pattern_size, square_size_mm):
    cols, rows = pattern_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_mm
    return objp


def main():
    cfg = load_config()
    cols, rows = tuple(cfg["board"]["pattern_size"])      # inner corners
    sq_mm = float(cfg["board"]["square_size_mm"])
    s_m = sq_mm * 1e-3
    cam = cfg["camera"]
    camera_file = cfg["calib"]["file"]
    rb_file = cfg["robot_calib"]["file"]
    z_board = float(cfg["robot_calib"]["z_board_m"])

    disp = cfg.get("display", {})
    stride = int(disp.get("label_stride", 1))
    unit = str(disp.get("label_unit", "m"))
    decimals = int(disp.get("label_decimals", 3))
    font_scale = float(disp.get("label_font_scale", 0.4))

    if not os.path.exists(camera_file):
        print(f"Missing {camera_file}.", file=sys.stderr); sys.exit(1)
    if not os.path.exists(rb_file):
        print(f"Missing {rb_file}.", file=sys.stderr); sys.exit(1)

    cam_data = np.load(camera_file)
    K, dist = cam_data["K"], cam_data["dist"]
    rb_data = np.load(rb_file)
    R_br = rb_data["R"]
    t_br = rb_data["t"]
    theta = float(rb_data["theta"])

    # Outer grid: i = 0..(cols+1), j = 0..(rows+1)
    n_outer_i = cols + 2     # e.g. 15
    n_outer_j = rows + 2     # e.g. 11
    board_w_mm = (n_outer_i - 1) * sq_mm   # 14 * 20 = 280
    board_h_mm = (n_outer_j - 1) * sq_mm   # 10 * 20 = 200
    W = int(round(board_w_mm * PIXELS_PER_MM))
    H = int(round(board_h_mm * PIXELS_PER_MM))

    # solvePnP uses the inner-corner frame, so outer (i, j) -> ((i-1)*sq, (j-1)*sq, 0) mm
    def outer_mm(i, j):
        return ((i - 1) * sq_mm, (j - 1) * sq_mm, 0.0)

    i_max = n_outer_i - 1
    j_max = n_outer_j - 1
    outer_corners_3d = np.float32([
        outer_mm(0,     0),       # board (0, 0)    bottom-left
        outer_mm(i_max, 0),       # board (14, 0)   bottom-right
        outer_mm(i_max, j_max),   # board (14, 10)  top-right
        outer_mm(0,     j_max),   # board (0, 10)   top-left
    ])

    # Destination quad. Image y is DOWN, so board y=0 maps to image bottom.
    dst_quad = np.float32([
        [0,     H - 1],
        [W - 1, H - 1],
        [W - 1, 0],
        [0,     0],
    ])

    objp = build_inner_object_points((cols, rows), sq_mm)

    # Precompute pixel position + robot xyz for every outer grid corner.
    grid_ij = []
    grid_uv = []
    grid_robot = []
    for j in range(n_outer_j):
        for i in range(n_outer_i):
            u = int(round(i * sq_mm * PIXELS_PER_MM))
            v = int(round((H - 1) - j * sq_mm * PIXELS_PER_MM))
            r_xy = R_br @ np.array([i * s_m, j * s_m]) + t_br
            grid_ij.append((i, j))
            grid_uv.append((u, v))
            grid_robot.append((r_xy[0], r_xy[1], z_board))

    cap = cv2.VideoCapture(int(cam["index"]))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cam["fourcc"])))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam["width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam["height"]))
    cap.set(cv2.CAP_PROP_FPS, int(cam["fps"]))
    if not cap.isOpened():
        print("Cannot open webcam.", file=sys.stderr); sys.exit(1)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                  | cv2.CALIB_CB_NORMALIZE_IMAGE
                  | cv2.CALIB_CB_FAST_CHECK)

    print(f"[ar_topdown] canvas {W}x{H}px  ({PIXELS_PER_MM}px/mm)  q=quit")
    last_t = time.time()
    fps = 0.0
    scale_disp = 1.0 if unit == "m" else 1000.0
    last_warp = np.zeros((H, W, 3), np.uint8)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), find_flags)

        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, dist,
                                              flags=cv2.SOLVEPNP_ITERATIVE)
            if ok_pnp:
                src_quad, _ = cv2.projectPoints(outer_corners_3d, rvec, tvec, K, dist)
                src_quad = src_quad.reshape(-1, 2).astype(np.float32)
                Hmat = cv2.getPerspectiveTransform(src_quad, dst_quad)
                last_warp = cv2.warpPerspective(frame, Hmat, (W, H))

        canvas = last_warp.copy()

        for (i, j), (u, v), (rx, ry, _rz) in zip(grid_ij, grid_uv, grid_robot):
            if (i % stride) or (j % stride):
                continue
            cv2.circle(canvas, (u, v), 3, (0, 255, 255), -1, cv2.LINE_AA)
            label = f"({rx * scale_disp:.{decimals}f},{ry * scale_disp:.{decimals}f})"
            # black outline + white text for readability over chessboard
            cv2.putText(canvas, label, (u + 3, v - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                        (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(canvas, label, (u + 3, v - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                        (255, 255, 255), 1, cv2.LINE_AA)

        now = time.time()
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        hud = (f"{fps:5.1f} FPS  detected={found}  "
               f"unit={unit}  z={z_board:.4f}m  theta={np.degrees(theta):+.1f}deg")
        cv2.putText(canvas, hud, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, hud, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("ar_topdown", canvas)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
