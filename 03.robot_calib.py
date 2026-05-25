"""Solve the 2D rigid transform from the board frame B to the robot base R.

B frame: origin = outer bottom-left corner of the board, +X along the long side,
         +Y along the short side, units in meters.
R frame: robot base, units in meters.

Inputs come from config.yaml -> robot_calib.touch_points (just edit the YAML).
Each entry has board outer-grid index (i, j) and the measured robot (x, y, z).

Usage:
    python 03.robot_calib.py          # solve and save
    python 03.robot_calib.py show     # print current points + residuals (no save)
"""
import os
import sys
import numpy as np
import yaml


def load_config():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "config.yaml"), "r") as f:
        return yaml.safe_load(f)


def procrustes_2d(P_B, P_R):
    """Best-fit rigid (rotation+translation, scale=1) mapping P_B -> P_R.

    Returns (theta, t, R_mat, residuals_mm).
    """
    cB = P_B.mean(axis=0)
    cR = P_R.mean(axis=0)
    qB = P_B - cB
    qR = P_R - cR
    H = qB.T @ qR
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(2)
    if np.linalg.det(Vt.T @ U.T) < 0:
        D[1, 1] = -1
    R_mat = Vt.T @ D @ U.T
    t = cR - R_mat @ cB
    theta = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))

    pred = (R_mat @ P_B.T).T + t
    err = np.linalg.norm(pred - P_R, axis=1)  # meters
    return theta, t, R_mat, err


def main():
    cfg = load_config()
    s = float(cfg["board"]["square_size_mm"]) * 1e-3  # m
    pts = cfg["robot_calib"]["touch_points"]
    out_file = cfg["robot_calib"]["file"]
    z_board = float(cfg["robot_calib"]["z_board_m"])

    if len(pts) < 2:
        print("Need at least 2 touch_points.", file=sys.stderr)
        sys.exit(1)

    P_B = np.array([[p["i"] * s, p["j"] * s] for p in pts], dtype=np.float64)
    P_R = np.array([[p["x"], p["y"]] for p in pts], dtype=np.float64)
    z_meas = np.array([p["z"] for p in pts], dtype=np.float64)

    theta, t, R_mat, err = procrustes_2d(P_B, P_R)

    print(f"[robot_calib] points: {len(pts)}")
    print(f"[robot_calib] theta = {np.degrees(theta):+.3f} deg")
    print(f"[robot_calib] t     = [{t[0]:+.4f}, {t[1]:+.4f}] m")
    print(f"[robot_calib] z_board_m (cfg) = {z_board:.4f}, "
          f"z measured mean = {z_meas.mean():.4f}, std = {z_meas.std()*1000:.2f} mm")
    print("[robot_calib] per-point residuals (mm):")
    for p, e in zip(pts, err):
        print(f"    (i={p['i']:>2}, j={p['j']:>2})  err = {e*1000:6.2f} mm")
    print(f"[robot_calib] RMS = {np.sqrt((err**2).mean())*1000:.2f} mm  "
          f"max = {err.max()*1000:.2f} mm")

    mode = sys.argv[1] if len(sys.argv) > 1 else "save"
    if mode == "show":
        print("[robot_calib] show-only, not saving.")
        return

    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    np.savez(out_file,
             theta=theta,
             t=t,
             R=R_mat,
             z_board=z_board,
             residuals_m=err,
             points_B=P_B,
             points_R=P_R)
    print(f"[robot_calib] saved -> {out_file}")


if __name__ == "__main__":
    main()
