"""05.estimate_object_cord.py
Same two-window layout as 04.ar_final.py, plus YOLO object detection.

For every detected object, a dot is drawn at the bounding-box center:
  - on the LEFT camera view (yellow dot)
  - on the RIGHT top-down view (via the locked homography)
Next to the dot on the top-down view, the robot-base (x, y) coordinate is shown.

Run only after the pose is locked (chessboard measured), otherwise dots show
only on the camera view.
"""
import os
import sys
import time
import numpy as np
import cv2
import yaml
from ultralytics import YOLO


PIXELS_PER_MM = 6
LOCK_FRAMES   = 10

# YOLO settings (same weights as /home/khw/workspace/yolo/04.inference.py)
YOLO_WEIGHTS = "/home/khw/workspace/yolo/outputs/runs/green_cube_v1/weights/best.pt"
YOLO_CONF    = 0.25
YOLO_IOU     = 0.45
YOLO_DEVICE  = 0


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


def build_box_points(pattern_size, square_size_mm, height_mm):
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
    overlay = img.copy()
    cv2.fillPoly(overlay, [top], color_z)
    cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)
    cv2.drawContours(img, [bottom], -1, color_x, 2, cv2.LINE_AA)
    cv2.drawContours(img, [top],    -1, color_z, 2, cv2.LINE_AA)
    for i in range(4):
        cv2.line(img, tuple(bottom[i]), tuple(top[i]), color_y, 2, cv2.LINE_AA)


def main():
    cfg = load_config()
    cols, rows = tuple(cfg["board"]["pattern_size"])
    sq_mm = float(cfg["board"]["square_size_mm"])
    s_m = sq_mm * 1e-3
    box_height_mm = float(cfg["board"].get(
        "box_height_mm", cfg["board"]["axis_length_mm"]))
    color_x = tuple(cfg["colors"]["x"])
    color_y = tuple(cfg["colors"]["y"])
    color_z = tuple(cfg["colors"]["z"])
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
        print(f"Missing {camera_file}. Run 01.calibrate.py first.", file=sys.stderr); sys.exit(1)
    if not os.path.exists(rb_file):
        print(f"Missing {rb_file}. Run 03.robot_calib.py first.", file=sys.stderr); sys.exit(1)
    if not os.path.exists(YOLO_WEIGHTS):
        print(f"Missing YOLO weights: {YOLO_WEIGHTS}", file=sys.stderr); sys.exit(1)

    cam_data = np.load(camera_file)
    K, dist = cam_data["K"], cam_data["dist"]
    rb_data = np.load(rb_file)
    R_br = rb_data["R"]
    t_br = rb_data["t"]
    theta = float(rb_data["theta"])
    print(f"[07] camera RMS={float(cam_data['rms']):.3f}px  "
          f"theta={np.degrees(theta):+.2f}deg")

    print(f"[07] loading YOLO weights: {YOLO_WEIGHTS}")
    model = YOLO(YOLO_WEIGHTS)
    class_names = model.names

    objp = build_inner_object_points((cols, rows), sq_mm)
    box_3d = build_box_points((cols, rows), sq_mm, box_height_mm)

    n_outer_i = cols + 2
    n_outer_j = rows + 2
    board_w_mm = (n_outer_i - 1) * sq_mm
    board_h_mm = (n_outer_j - 1) * sq_mm
    Wt = int(round(board_w_mm * PIXELS_PER_MM))
    Ht = int(round(board_h_mm * PIXELS_PER_MM))

    def outer_mm(i, j):
        return ((i - 1) * sq_mm, (j - 1) * sq_mm, 0.0)

    i_max = n_outer_i - 1
    j_max = n_outer_j - 1
    outer_corners_3d = np.float32([
        outer_mm(0,     0),
        outer_mm(i_max, 0),
        outer_mm(i_max, j_max),
        outer_mm(0,     j_max),
    ])
    dst_quad = np.float32([
        [0,      Ht - 1],
        [Wt - 1, Ht - 1],
        [Wt - 1, 0],
        [0,      0],
    ])

    grid_uv = []
    grid_robot = []
    for j in range(n_outer_j):
        for i in range(n_outer_i):
            u = int(round(i * sq_mm * PIXELS_PER_MM))
            v = int(round((Ht - 1) - j * sq_mm * PIXELS_PER_MM))
            r_xy = R_br @ np.array([i * s_m, j * s_m]) + t_br
            grid_uv.append((i, j, u, v))
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

    print(f"[07] measuring first {LOCK_FRAMES} frames then locking pose. "
          f"q=quit  r=re-measure")
    last_t = time.time()
    fps = 0.0
    scale_disp = 1.0 if unit == "m" else 1000.0

    rvec_samples = []
    tvec_samples = []
    locked_box_imgpts = None
    locked_Hmat = None

    def draw_labels(canvas):
        for (i, j, u, v), (rx, ry, _rz) in zip(grid_uv, grid_robot):
            if (i % stride) or (j % stride):
                continue
            cv2.circle(canvas, (u, v), 3, (0, 255, 255), -1, cv2.LINE_AA)
            label = f"({rx * scale_disp:.{decimals}f},{ry * scale_disp:.{decimals}f})"
            cv2.putText(canvas, label, (u + 3, v - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                        (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(canvas, label, (u + 3, v - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                        (255, 255, 255), 1, cv2.LINE_AA)

    def topdown_px_to_robot(u, v):
        """Convert a pixel in the rectified top-down canvas to robot (x, y, z) m."""
        i_m = (u / PIXELS_PER_MM) * 1e-3
        j_m = ((Ht - 1 - v) / PIXELS_PER_MM) * 1e-3
        r_xy = R_br @ np.array([i_m, j_m]) + t_br
        return float(r_xy[0]), float(r_xy[1]), z_board

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # YOLO inference on the clean frame (before any overlay).
        yolo_res = model.predict(
            frame, conf=YOLO_CONF, iou=YOLO_IOU,
            device=YOLO_DEVICE, verbose=False,
        )[0]
        objects = []   # list of dict: cx, cy, cls, conf
        if yolo_res.boxes is not None and len(yolo_res.boxes):
            xyxy = yolo_res.boxes.xyxy.cpu().numpy()
            confs = yolo_res.boxes.conf.cpu().numpy()
            clss = yolo_res.boxes.cls.cpu().numpy().astype(int)
            for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clss):
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                objects.append({"cx": cx, "cy": cy, "conf": float(c),
                                "cls": class_names.get(int(k), str(int(k)))})

        # Chessboard pose lock (one-shot, then frozen).
        if locked_Hmat is None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), find_flags)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, dist,
                                                  flags=cv2.SOLVEPNP_ITERATIVE)
                if ok_pnp:
                    rvec_samples.append(rvec.reshape(3))
                    tvec_samples.append(tvec.reshape(3))
                    if len(rvec_samples) >= LOCK_FRAMES:
                        rvec_mean = np.mean(rvec_samples, axis=0).reshape(3, 1)
                        tvec_mean = np.mean(tvec_samples, axis=0).reshape(3, 1)
                        imgpts_box, _ = cv2.projectPoints(box_3d, rvec_mean, tvec_mean, K, dist)
                        src_quad, _ = cv2.projectPoints(outer_corners_3d,
                                                        rvec_mean, tvec_mean, K, dist)
                        src_quad = src_quad.reshape(-1, 2).astype(np.float32)
                        locked_box_imgpts = imgpts_box
                        locked_Hmat = cv2.getPerspectiveTransform(src_quad, dst_quad)
                        print(f"[07] pose locked after {LOCK_FRAMES} samples.")

        now = time.time(); dt = now - last_t; last_t = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

        # RIGHT: warp clean frame
        if locked_Hmat is not None:
            right = cv2.warpPerspective(frame, locked_Hmat, (Wt, Ht))
            draw_labels(right)
            # Project each object center into the top-down view and label it.
            if objects:
                pts_src = np.array([[[o["cx"], o["cy"]]] for o in objects], dtype=np.float32)
                pts_dst = cv2.perspectiveTransform(pts_src, locked_Hmat).reshape(-1, 2)
                for o, (u, v) in zip(objects, pts_dst):
                    o["u_td"] = float(u); o["v_td"] = float(v)
                    rx, ry, rz = topdown_px_to_robot(u, v)
                    o["rx"], o["ry"], o["rz"] = rx, ry, rz
                    iu, iv = int(round(u)), int(round(v))
                    if 0 <= iu < Wt and 0 <= iv < Ht:
                        cv2.circle(right, (iu, iv), 6, (0, 0, 255), -1, cv2.LINE_AA)
                        cv2.circle(right, (iu, iv), 6, (255, 255, 255), 1, cv2.LINE_AA)
                        text = f"{o['cls']} ({rx * scale_disp:.{decimals}f},{ry * scale_disp:.{decimals}f})"
                        cv2.putText(right, text, (iu + 8, iv - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 0, 0), 3, cv2.LINE_AA)
                        cv2.putText(right, text, (iu + 8, iv - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 0, 255), 1, cv2.LINE_AA)
            hud_r = (f"unit={unit}  z={z_board:.4f}m  "
                     f"theta={np.degrees(theta):+.1f}deg  [LOCKED]  det={len(objects)}")
        else:
            right = np.zeros((Ht, Wt, 3), np.uint8)
            hud_r = f"measuring {len(rvec_samples)}/{LOCK_FRAMES}...  det={len(objects)}"
        cv2.putText(right, hud_r, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(right, hud_r, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 1, cv2.LINE_AA)

        # LEFT: live camera + locked cuboid + object dots
        left = frame
        if locked_box_imgpts is not None:
            draw_cuboid(left, locked_box_imgpts, color_x, color_y, color_z)
            status = "LOCKED"
        else:
            status = f"measuring {len(rvec_samples)}/{LOCK_FRAMES}"
        for o in objects:
            cx, cy = int(round(o["cx"])), int(round(o["cy"]))
            cv2.circle(left, (cx, cy), 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(left, (cx, cy), 6, (255, 255, 255), 1, cv2.LINE_AA)
            text = o["cls"]
            if "rx" in o:
                text = (f"{o['cls']} "
                        f"({o['rx'] * scale_disp:.{decimals}f},"
                        f"{o['ry'] * scale_disp:.{decimals}f})")
            cv2.putText(left, text, (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(left, text, (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(left, f"{fps:5.1f} FPS  {status}  det={len(objects)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("ar_axes", left)
        cv2.imshow("ar_topdown", right)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('r'):
            rvec_samples.clear()
            tvec_samples.clear()
            locked_box_imgpts = None
            locked_Hmat = None
            print("[07] re-measuring.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
