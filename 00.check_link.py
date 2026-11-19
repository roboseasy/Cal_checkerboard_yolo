"""Smoke test for the camera link — run this before 01.

Opens whatever `camera.source` points at, reports the frame size and the
measured frame rate, and (for the lekiwi source) lists the cameras the host is
publishing plus the arm joint positions riding along in the same message.

Kits do not agree on which physical camera is called "front" — on some the two
names are swapped — so this asks which one to look at instead of trusting
config.yaml. In the live view `n` switches between them; every observation
carries all cameras, so switching costs nothing.

If the stored calibration was made at a different resolution it says so, since
that silently ruins every pose estimate downstream.

    python 00.check_link.py                 # 카메라를 고르고 라이브 창, q 로 종료
    python 00.check_link.py --camera wrist  # 묻지 않고 바로 wrist
    python 00.check_link.py --no-gui        # 텍스트만
"""
import os
import sys
import time

import numpy as np
import cv2

from config_util import load_config, resolve_path
from frame_source import ANY_CAMERA, FrameSourceError, LeKiwiStream, open_frame_source

PROBE_SECONDS = 3.0


def parse_args(argv):
    """Tiny hand-rolled parser: --no-gui and --camera <name>."""
    show_gui = "--no-gui" not in argv
    camera = None
    if "--camera" in argv:
        i = argv.index("--camera")
        if i + 1 >= len(argv):
            print("--camera needs a camera name, e.g. --camera wrist", file=sys.stderr)
            sys.exit(2)
        camera = argv[i + 1]
    return show_gui, camera


def choose_camera(names, default):
    """Ask which camera to watch, falling back to `default` when not a terminal."""
    fallback = default if default in names else names[0]
    if len(names) == 1:
        return names[0]
    if not sys.stdin.isatty():
        return fallback

    print("[check] host 가 발행 중인 카메라:")
    for n, name in enumerate(names, 1):
        tag = "   <- config.yaml 의 camera.name" if name == default else ""
        print(f"    {n}) {name}{tag}")

    while True:
        try:
            raw = input(f"카메라를 고르세요 [번호 또는 이름, Enter={fallback}]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return fallback
        if not raw:
            return fallback
        if raw.isdigit() and 1 <= int(raw) <= len(names):
            return names[int(raw) - 1]
        if raw in names:
            return raw
        print(f"  '{raw}' 는 없는 카메라입니다. {names} 중에서 고르세요.")


def next_camera(cap):
    """Cycle to the camera after the current one."""
    i = cap.camera_names.index(cap.cam_name)
    return cap.camera_names[(i + 1) % len(cap.camera_names)]


def main():
    show_gui, requested = parse_args(sys.argv[1:])
    cfg = load_config()
    configured = cfg.get("camera", {}).get("name", "front")

    # For the lekiwi source, connect without committing to a camera so the list
    # can be shown even when config.yaml names one the host does not publish.
    is_lekiwi = str(cfg.get("camera", {}).get("source", "local")).lower() == "lekiwi"
    try:
        cap = open_frame_source(cfg, cam_name=ANY_CAMERA if is_lekiwi else requested)
    except FrameSourceError as e:
        print(f"[check] FAILED to open the camera.\n{e}", file=sys.stderr)
        sys.exit(1)

    if isinstance(cap, LeKiwiStream):
        print(f"[check] host: {cap.address}")
        print(f"[check] cameras published by the host: {cap.camera_names}")
        chosen = requested if requested else choose_camera(cap.camera_names, configured)
        try:
            cap.use_camera(chosen)
        except FrameSourceError as e:
            print(f"[check] {e}", file=sys.stderr)
            cap.release()
            sys.exit(1)

    print(f"[check] source: {cap.describe()}")

    ok, frame = cap.read()
    if not ok:
        print("[check] connected but no frame arrived.", file=sys.stderr)
        cap.release()
        sys.exit(1)

    h, w = frame.shape[:2]
    print(f"[check] frame size: {w}x{h}")

    calib_file = resolve_path(cfg["calib"]["file"])
    if os.path.exists(calib_file):
        stored = np.load(calib_file)["image_size"]
        if tuple(int(v) for v in stored) != (w, h):
            print(f"[check] WARNING: {calib_file} was calibrated at "
                  f"{int(stored[0])}x{int(stored[1])}, but this camera gives "
                  f"{w}x{h}. Re-run 01.calibrate.py for this camera.")
        else:
            print(f"[check] {calib_file} matches this resolution.")
    else:
        print(f"[check] no {calib_file} yet — run 01.calibrate.py next.")

    # Frame rate over a short probe window, plus board visibility as a bonus.
    pattern_size = tuple(cfg["board"]["pattern_size"])
    find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                  | cv2.CALIB_CB_NORMALIZE_IMAGE
                  | cv2.CALIB_CB_FAST_CHECK)

    frames = 0
    boards = 0
    start = time.time()
    while time.time() - start < PROBE_SECONDS:
        ok, frame = cap.read()
        if not ok:
            print("[check] stream dropped mid-probe.", file=sys.stderr)
            break
        frames += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, _ = cv2.findChessboardCorners(gray, pattern_size, find_flags)
        boards += int(found)
    elapsed = time.time() - start
    if elapsed > 0:
        print(f"[check] {frames} frames in {elapsed:.1f}s -> {frames / elapsed:.1f} FPS")
    print(f"[check] chessboard {pattern_size[0]}x{pattern_size[1]} seen in "
          f"{boards}/{frames} frames")

    if isinstance(cap, LeKiwiStream) and cap.state:
        print("[check] robot state in the same message:")
        for key in sorted(cap.state):
            print(f"    {key:24s} {cap.state[key]}")

    if show_gui:
        can_switch = isinstance(cap, LeKiwiStream) and len(cap.camera_names) > 1
        print("[check] live view — q to quit" + ("  |  n to switch camera" if can_switch else ""))
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[check] stream dropped.", file=sys.stderr)
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern_size, find_flags)
            if found:
                cv2.drawChessboardCorners(frame, pattern_size, corners, found)
            fh, fw = frame.shape[:2]
            name = getattr(cap, "cam_name", "camera")
            hud = f"{name}  {fw}x{fh}  board={found}"
            if can_switch:
                hud += "  [n=switch]"
            cv2.putText(frame, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, hud, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("check_link", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('n') and can_switch:
                cap.use_camera(next_camera(cap))
                print(f"[check] switched to '{cap.cam_name}'")
        cv2.destroyAllWindows()

    if isinstance(cap, LeKiwiStream) and cap.cam_name != configured:
        print(f"[check] NOTE: config.yaml 의 camera.name 은 '{configured}' 입니다. "
              f"01~05 도 '{cap.cam_name}' 를 쓰려면 그 값을 바꾸세요.")

    cap.release()
    print("[check] done.")


if __name__ == "__main__":
    main()
