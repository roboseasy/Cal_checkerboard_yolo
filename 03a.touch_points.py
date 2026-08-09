"""Collect robot_calib.touch_points into config.yaml — step 3 helper.

Walks through the outer-grid points that need touching, takes the end-effector
(x, y, z) you read off the arm for each one, checks that the set is actually
solvable, and writes the block into config.yaml without disturbing the rest of
the file.

Getting the EE coordinate itself is not this repo's job: use the tool that can
free-drive the arm and display the end-effector pose in metres (the Physical
Labs EEF pane, for instance). Note that free-drive needs the motor bus, so
lekiwi_host must be stopped while you measure — step 3 uses no camera anyway.

Run `python 02b.board_index.py` first if you are not sure which physical corner
carries which (i, j).

    python 03a.touch_points.py
"""
import os
import re
import subprocess
import sys

import yaml

from config_util import BASE_DIR, CONFIG_PATH, load_config

# The spread suggested by the README: the origin plus points that give a
# baseline along both the long (i) and the short (j) side of the board.
DEFAULT_POINTS = (
    (0, 0, "원점 — 외곽 좌하단 코너"),
    (1, 1, "좌하단 흰칸 우상"),
    (1, 4, "좌하단에서 위 4번째 검은칸 우상"),
    (13, 1, "우하단 검은칸 좌상"),
    (13, 4, "우하단에서 위 4번째 흰칸 좌상"),
)

PLAUSIBLE_XY_M = 1.0      # an SO-101 sized arm cannot reach further than this
PLAUSIBLE_Z_M = 1.0


def parse_xyz(raw):
    """'0.24 0.08 0.08' or '0.24, 0.08, 0.08' -> (x, y, z), or None."""
    parts = raw.replace(",", " ").split()
    if len(parts) != 3:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def looks_implausible(xyz):
    """Return a complaint string when the numbers cannot be a real EE pose."""
    x, y, z = xyz
    if max(abs(x), abs(y)) > PLAUSIBLE_XY_M:
        return f"x 또는 y 가 {PLAUSIBLE_XY_M} m 를 넘습니다. 단위가 mm 아닌가요?"
    if not (0.0 <= z <= PLAUSIBLE_Z_M):
        return f"z 가 0~{PLAUSIBLE_Z_M} m 범위 밖입니다. 단위가 mm 아닌가요?"
    return None


def ask_point(i, j, note, index, total):
    """Prompt until a usable xyz arrives. Returns None when skipped."""
    print(f"\n[{index}/{total}] i={i}, j={j}   ({note})")
    while True:
        try:
            raw = input("  EE 를 이 점에 대고 x y z 입력 (m) "
                        "[s=건너뛰기, q=저장없이 종료]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return "quit"
        if raw.lower() == "q":
            return "quit"
        if raw.lower() == "s":
            print("  건너뜀")
            return None
        if not raw:
            continue
        xyz = parse_xyz(raw)
        if xyz is None:
            print("  숫자 3개를 공백이나 콤마로 구분해 입력하세요. 예: 0.2483 0.0833 0.0804")
            continue
        complaint = looks_implausible(xyz)
        if complaint:
            print(f"  경고: {complaint}")
            try:
                if input("  그래도 쓸까요? [y/N]: ").strip().lower() != "y":
                    continue
            except (EOFError, KeyboardInterrupt):
                print()
                return "quit"
        print(f"  기록: x={xyz[0]:+.4f}  y={xyz[1]:+.4f}  z={xyz[2]:.4f}")
        return xyz


def ask_extra_points():
    """Let the user add indices beyond the default list."""
    extra = []
    while True:
        try:
            raw = input("\n추가로 찍을 점의 인덱스 'i j' [Enter=그만]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return extra
        if not raw:
            return extra
        parts = raw.replace(",", " ").split()
        if len(parts) != 2 or not all(p.lstrip("-").isdigit() for p in parts):
            print("  'i j' 형식으로 입력하세요. 예: 7 8")
            continue
        i, j = int(parts[0]), int(parts[1])
        extra.append((i, j, "추가 측정점"))


def check_spread(entries):
    """Report whether the points can pin down a 2D rigid transform."""
    problems = []
    if len(entries) < 2:
        problems.append("점이 2개 미만입니다. 최소 2점, 3점 이상 권장.")
        return problems
    i_vals = {e[0] for e in entries}
    j_vals = {e[1] for e in entries}
    i_span = max(i_vals) - min(i_vals)
    j_span = max(j_vals) - min(j_vals)
    print(f"\n  i 방향 베이스라인: {i_span}칸  ({sorted(i_vals)})")
    print(f"  j 방향 베이스라인: {j_span}칸  ({sorted(j_vals)})")
    if i_span == 0:
        problems.append("모든 점의 i 가 같습니다 — 한 줄에 몰려 있어 회전이 결정되지 않습니다.")
    if j_span == 0:
        problems.append("모든 점의 j 가 같습니다 — 한 줄에 몰려 있어 회전이 결정되지 않습니다.")
    if i_span and i_span < 4:
        problems.append(f"i 베이스라인이 {i_span}칸으로 짧습니다. 측정 오차가 각도로 증폭됩니다.")
    if j_span and j_span < 3:
        problems.append(f"j 베이스라인이 {j_span}칸으로 짧습니다. 측정 오차가 각도로 증폭됩니다.")
    if len(entries) < 3:
        problems.append("점이 2개뿐이라 잔차 RMS 를 볼 수 없습니다. 3점 이상 권장.")
    return problems


def replace_touch_points(text, entries):
    """Swap the touch_points block, leaving every other byte of the file alone.

    PyYAML round-tripping would drop all the comments in config.yaml, so this
    edits the text directly.
    """
    lines = text.splitlines(keepends=True)
    start = None
    indent = ""
    for n, line in enumerate(lines):
        m = re.match(r"^(\s*)touch_points\s*:", line)
        if m:
            start = n
            indent = m.group(1)
            break
    if start is None:
        raise ValueError("config.yaml 에서 touch_points 항목을 찾지 못했습니다.")

    # Consume the existing list items (if any) that belong to this key.
    end = start + 1
    while end < len(lines) and re.match(rf"^{re.escape(indent)}\s+-\s", lines[end]):
        end += 1

    block = [f"{indent}touch_points:\n"]
    for i, j, x, y, z, note in entries:
        block.append(f"{indent}  - {{i: {i:<2}, j: {j:<2}, "
                     f"x: {x:+.4f}, y: {y:+.4f}, z: {z:.4f}}}   # {note}\n")
    return "".join(lines[:start] + block + lines[end:])


def replace_z_board(text, value):
    """Update robot_calib.z_board_m in place, keeping its trailing comment."""
    def sub(m):
        return f"{m.group(1)}{value:.4f}{m.group(3)}"
    new, count = re.subn(r"^(\s*z_board_m\s*:\s*)([-\d.eE+]+)(.*)$",
                         sub, text, count=1, flags=re.MULTILINE)
    return new if count else text


def main():
    cfg = load_config()
    existing = cfg.get("robot_calib", {}).get("touch_points") or []
    if existing:
        print(f"[touch] config.yaml 에 이미 {len(existing)}개의 touch_points 가 있습니다. "
              "계속하면 덮어씁니다.")

    print("[touch] 측정 전 확인:")
    print("  - lekiwi_host 를 내렸는지 (freedrive 는 모터 버스를 직접 잡습니다)")
    print("  - 보드와 로봇 베이스가 측정 내내 고정되어 있는지")
    print("  - 어느 코너가 (0,0) 인지 — 모르면 02b.board_index.py 로 먼저 확인")

    targets = list(DEFAULT_POINTS)
    collected = []
    quit_early = False

    for n, (i, j, note) in enumerate(targets, 1):
        result = ask_point(i, j, note, n, len(targets))
        if result == "quit":
            quit_early = True
            break
        if result is not None:
            collected.append((i, j, result[0], result[1], result[2], note))

    if not quit_early:
        for i, j, note in ask_extra_points():
            result = ask_point(i, j, note, len(collected) + 1, len(collected) + 1)
            if result == "quit":
                break
            if result is not None:
                collected.append((i, j, result[0], result[1], result[2], note))

    if not collected:
        print("\n[touch] 기록된 점이 없습니다. config.yaml 은 그대로 둡니다.")
        return

    print("\n" + "=" * 60)
    print(f"  수집된 점: {len(collected)}개")
    for i, j, x, y, z, _ in collected:
        print(f"    (i={i:>2}, j={j:>2})   x={x:+.4f}  y={y:+.4f}  z={z:.4f}")

    problems = check_spread(collected)
    if problems:
        print("\n  ⚠ 확인이 필요합니다:")
        for p in problems:
            print(f"    - {p}")

    try:
        if input("\nconfig.yaml 에 저장할까요? [Y/n]: ").strip().lower() == "n":
            print("[touch] 저장하지 않았습니다.")
            return
    except (EOFError, KeyboardInterrupt):
        print("\n[touch] 저장하지 않았습니다.")
        return

    with open(CONFIG_PATH, "r") as f:
        text = f.read()

    new_text = replace_touch_points(text, collected)

    z_mean = sum(e[4] for e in collected) / len(collected)
    z_spread_mm = (max(e[4] for e in collected) - min(e[4] for e in collected)) * 1000
    print(f"\n  측정된 z 평균 = {z_mean:.4f} m  (편차 {z_spread_mm:.1f} mm)")
    try:
        if input(f"  z_board_m 을 {z_mean:.4f} 로 갱신할까요? [Y/n]: ").strip().lower() != "n":
            new_text = replace_z_board(new_text, z_mean)
    except (EOFError, KeyboardInterrupt):
        print()

    # Refuse to write anything that would not parse back.
    try:
        reparsed = yaml.safe_load(new_text)
        written = reparsed["robot_calib"]["touch_points"]
        assert len(written) == len(collected)
    except Exception as e:
        print(f"[touch] 생성된 YAML 이 올바르지 않아 저장을 취소합니다: {e}", file=sys.stderr)
        sys.exit(1)

    backup = CONFIG_PATH + ".bak"
    with open(backup, "w") as f:
        f.write(text)
    with open(CONFIG_PATH, "w") as f:
        f.write(new_text)
    print(f"[touch] saved -> {CONFIG_PATH}  (이전 내용은 {os.path.basename(backup)})")

    print("\n[touch] 03.robot_calib.py 로 풀어봅니다 (저장 없이 확인만)...\n")
    subprocess.run([sys.executable, os.path.join(BASE_DIR, "03.robot_calib.py"), "show"])
    print("\n[touch] 잔차가 만족스러우면:  python 03.robot_calib.py")


if __name__ == "__main__":
    main()
