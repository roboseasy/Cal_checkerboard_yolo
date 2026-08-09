# Checkerboard ↔ Robot Coordinate Mapping & AR (LeKiwi 원격 버전)

> 이 브랜치(`lekiwi`)는 **LeKiwi 모바일 매니퓰레이터를 원격으로** 쓰기 위한 것입니다.
> 노트북에 웹캠을 직접 꽂는 SO-101 구성은 `main` 브랜치를 쓰세요.

웹캠으로 인쇄 체커보드를 잡아 **로봇 베이스 좌표계와 매핑**하고, 그 위에서
- 보드 위에 3D 직육면체 AR 오버레이를 띄우고
- 위에서 본 평면 뷰(top-down)에서 격자 교차점마다 로봇 좌표를 표기하고
- YOLO로 검출한 객체의 바운딩 박스 중심점을 로봇 좌표로 환산해 보여주는

데모입니다. 모든 설정은 [config.yaml](config.yaml)에서 관리됩니다.

---

## main 브랜치와 달라진 점

| | main (SO-101) | lekiwi (이 브랜치) |
|---|---|---|
| 영상 | 노트북에 꽂은 USB 웹캠 | 파이의 `lekiwi_host` 가 ZMQ로 보내는 스트림 |
| 영상 코드 | 각 스크립트가 `cv2.VideoCapture(index)` | [frame_source.py](frame_source.py) 한 군데로 통일 |
| 캘리브 결과 | `calib/*.npz` | `calib/lekiwi/*.npz` (main 것과 안 섞이게 분리) |
| 사전 점검 | 없음 | [00.check_link.py](00.check_link.py) |
| 추가 의존성 | — | `pyzmq` |

`lerobot` 를 `yolo` 환경에 깔 필요는 **없습니다.** LeKiwi host가 보내는 메시지는
그냥 JSON이라 `pyzmq` 만으로 받습니다.

---

## 네트워크 구성

랜선 직결 기준입니다.

```
노트북 10.42.0.1  ──(유선)──  LeKiwi(라즈베리파이) 10.42.0.61
                              :5556  observation (영상 + 팔 관절값)  ← 우리가 쓰는 것
                              :5555  command (팔/베이스 제어)        ← 안 씀
```

---

## 보드 사양
- A4 인쇄, 정사각형 한 변 **20mm**, 14×10 squares (= **13×9 내부 코너**, 15×11 외곽 교차점)
- 보드는 가로(긴 변) > 세로(짧은 변)로 두고 고정 사용
- 인쇄는 **실제 크기 100%** 로 출력 후, 자로 한 칸을 측정해 20mm 인지 확인 (다르면 `config.yaml`의 `square_size_mm` 보정)

좌표계 정의:
- **보드 프레임 B**: 원점 = 보드 좌하단 외곽 코너, +X = 가로(긴 변), +Y = 세로(짧은 변), z=0 보드면
- **로봇 프레임 R**: LeKiwi 팔의 베이스
- z축은 두 프레임이 평행하다고 가정 (보드를 바닥/책상 위에 평평히 둠)

---

## 설치

노트북(클라이언트) 쪽:
```bash
conda create -n yolo python=3.12 -y
conda activate yolo
pip install -r requirements.txt
```
- `opencv-python`, `numpy`, `PyYAML`, `pyzmq`, `ultralytics` (YOLO)

---

## 0) 로봇 host 띄우기 — 무조건 이것부터

**IP 확인 → SSH 접속 → `./start_lekiwi_host.sh` 실행.**
host가 안 떠 있으면 01~05 전부 한 줄도 못 돕니다.

### ① 유선랜 IP 알아내기

```bash
# 내 노트북의 유선 인터페이스 (여기선 eno1 = 10.42.0.1)
ip -4 addr show

# 같은 랜에 붙은 파이 찾기 — 아래 셋 중 아무거나
ping -c1 lekiwi07.local        # 로봇 호스트명을 알면 이게 제일 빠르다
ip neigh show dev eno1         # -> 10.42.0.61 lladdr d8:3a:dd:... REACHABLE
nmap -sn 10.42.0.0/24          # 위 둘로 안 나오면 대역 스캔
```

찾은 주소로 `ping` 이 되는지 확인하고, `config.yaml` 의 `lekiwi.remote_ip` 에 적습니다.

```bash
ping -c2 10.42.0.61
```

### ② SSH 접속

```bash
ssh roboseasy@10.42.0.61
```

### ③ 미리 만들어 둔 host 스크립트 실행

```bash
./start_lekiwi_host.sh
```

정상이면 이렇게 나옵니다.

```
[lekiwi] starting host: id=lekiwi01 run_time=14400s
[lekiwi] laptop should connect to this Pi's IP (ZMQ 5555/5556)
WARNING:root:No command available
WARNING:root:Command not received for more than 500 milliseconds. Stopping the base.
```

- `No command available` · `Stopping the base` 경고는 **정상입니다.** 이 저장소의
  스크립트는 영상만 받고 명령(5555)은 안 보내기 때문에, host가 계속 그렇게 알립니다.
- **이 SSH 창은 켜 둔 채로 두세요.** 닫으면 host도 같이 죽습니다.

노트북에서 열렸는지 확인:

```bash
python 00.check_link.py --no-gui
```

<details>
<summary>스크립트 없이 직접 띄우려면</summary>

```bash
/home/roboseasy/lerobot_venv/bin/python -m lerobot.robots.lekiwi.lekiwi_host \
    --robot.id=lekiwi01 \
    --host.connection_time_s=14400
```

카메라 장치를 기본값과 다르게 쓰려면 `--robot.cameras=...` 로 넘깁니다.
(실물 킷 예시: `front` = `/dev/video2`, `wrist` = `/dev/video0`)
</details>

주의할 점 셋:

1. **`--host.connection_time_s` 기본값은 30초입니다.** 안 주면 30초 만에 host가
   스스로 꺼집니다. `start_lekiwi_host.sh` 는 14400초(4시간)로 넣어 뒀습니다.
2. **소비자는 한 번에 하나만.** host의 observation 소켓은 ZMQ PUSH라서, 클라이언트가
   둘 붙으면 프레임을 번갈아 나눠 가집니다. Physical Labs 앱 · teleop · record ·
   이 저장소 스크립트 중 **하나만** 켜 두세요.
3. **파이의 카메라 장치도 하나만.** 다른 프로그램이 카메라를 잡고 있으면 host가
   못 엽니다. 앱의 카메라 뷰를 먼저 닫으세요.

---

## 실행 순서 (00 → 05)

### 0) `00.check_link.py` — 연결 점검 + 카메라 고르기
```bash
python 00.check_link.py                 # 카메라를 고르고 라이브 창 (q 종료, n 전환)
python 00.check_link.py --camera wrist  # 묻지 않고 바로 지정
python 00.check_link.py --no-gui        # 텍스트만
```

실행하면 host가 발행 중인 카메라를 세어서 물어봅니다.

```
[check] host 가 발행 중인 카메라:
    1) front   <- config.yaml 의 camera.name
    2) wrist
카메라를 고르세요 [번호 또는 이름, Enter=front]:
```

**킷마다 `front` 와 `wrist` 가 뒤바뀌어 꽂힌 경우가 있어서** 이름만 믿으면 안 됩니다.
라이브 창에서 **`n`** 을 누르면 두 카메라를 오가며 눈으로 확인할 수 있습니다
(관측 메시지 하나에 모든 카메라가 실려 오므로 재접속 없이 즉시 바뀝니다).
config.yaml 과 다른 카메라를 골랐으면 종료할 때 알려 줍니다 — 01~05 도 그 카메라를
쓰려면 `camera.name` 을 바꾸세요.

확인해 주는 것:
- host가 발행 중인 카메라 이름 목록 (`['front', 'wrist']`)
- 프레임 크기와 실제 FPS
- 저장된 캘리브레이션 해상도와 **지금 카메라 해상도가 같은지** (다르면 경고 — 이게 다르면
  아래 모든 단계가 조용히 틀립니다)
- 체커보드가 몇 프레임에서 잡히는지
- 같은 메시지에 실려 오는 팔 관절값

여기서 실패하면 01 이후는 볼 것도 없습니다. 아래 [문제 해결](#문제-해결)로.

> **베이스에 고정된 카메라를 쓰세요 (보통 `front`).** 팔에 달린 카메라(보통 `wrist`)는
> 팔이 움직이면 시점이 바뀌어서, 04/05의 포즈 고정(pose lock)이 그 순간 무의미해집니다.
> 이름이 아니라 **`n` 으로 실제 화면을 보고** 어느 쪽이 베이스 고정인지 판단하세요.

### 1) `01.calibrate.py` — 카메라 내부 파라미터 캘리브레이션 (카메라당 1회)
```bash
python 01.calibrate.py
```
- **main 브랜치의 `calib/camera_params.npz` 를 재활용하면 안 됩니다.** 노트북 웹캠과
  LeKiwi 카메라는 해상도(640×480 vs 1280×720)도 렌즈 왜곡도 다릅니다.
- 체커보드를 다양한 각도/거리에서 비추며 **SPACE** 로 캡처
- 키: `SPACE`=캡처, `r`=초기화, `ESC`=종료
- 목표 매수(`target_shots`, 기본 20장) 도달 시 `cv2.calibrateCamera` 실행
- RMS 재투영 오차가 < 1.0px 이면 양호
- 결과: `calib/lekiwi/camera_params.npz` (`K`, `dist`, `rms`, `image_size`)

> 카메라가 로봇에 붙어 있으니, 보드를 손에 들고 카메라 앞에서 각도를 바꿔 가며 찍는 게
> 편합니다. 보드가 화면 구석·기울어진 각도에도 오도록 골고루 담으세요.

### 2) `02.ar_axes.py` — 보드 위 직육면체 AR (검증용)
```bash
python 02.ar_axes.py
```
- `calib/lekiwi/camera_params.npz` 를 로드해 매 프레임 `findChessboardCorners → solvePnP`
- 보드 내부 코너 영역(12×8 격자) 위에 높이 `box_height_mm` 직육면체를 그림
- 색: 밑면=초록(X), 기둥=파랑(Y), 윗면=빨강(Z)
- `q` 종료

직육면체가 보드에 딱 붙어서 안 흔들리면 01이 잘 된 겁니다.

### 3) EE로 보드 코너 측정 → `config.yaml` 직접 편집
LeKiwi 팔의 EE(엔드 이펙터)로 보드 위 정해진 외곽 격자점을 찍어 로봇 베이스 좌표를
기록합니다. **SO-101에서 재던 값은 못 씁니다 — LeKiwi에서 다시 재세요.**

`config.yaml` 의 `robot_calib.touch_points` 리스트에 항목 추가:

```yaml
robot_calib:
  z_board_m: 0.0804           # 보드 표면 높이 (z=상수 가정)
  touch_points:
    - {i: 0,  j: 0,  x: ..., y: ..., z: ...}   # 좌하단 외곽 코너
    - {i: 1,  j: 1,  x: ..., y: ..., z: ...}   # 좌하단 흰칸 우상
    - {i: 1,  j: 4,  x: ..., y: ..., z: ...}   # 좌하단에서 위 4번째 검은칸 우상
    - {i: 13, j: 1,  x: ..., y: ..., z: ...}   # 우하단 검은칸 좌상
    - {i: 13, j: 4,  x: ..., y: ..., z: ...}   # 우하단에서 위 4번째 흰칸 좌상
```
- 외곽 격자 인덱스: `i = 0..14` (X 방향), `j = 0..10` (Y 방향)
- 단위는 m
- X(긴 변)·Y(짧은 변) **양방향 모두에서 베이스라인을 확보**하도록 분포 (모든 점이 같은 행/열에 몰리면 안 됨)
- 최소 2점, 잔차 RMS 확인을 위해 3점 이상 권장

> 팔을 조그해서 EE 좌표(x, y, z)를 읽는 건 이 저장소 밖의 일입니다.
> Physical Labs 앱의 EEF 패널처럼 EE 위치를 m 단위로 보여주는 도구를 쓰세요.
> 이때 **앱이 host의 스트림을 먹고 있으므로**, 측정이 끝나면 앱을 닫고 04/05로 넘어가세요.

### 4) `03.robot_calib.py` — 보드 ↔ 로봇 2D 강체 변환 추정
```bash
python 03.robot_calib.py        # 풀고 저장
python 03.robot_calib.py show   # 풀어보기만 (저장 X)
```
- 카메라를 안 쓰는 유일한 단계라 원격이든 아니든 동일합니다
- yaml의 `touch_points` 로부터 Procrustes(SVD)로 (θ, t) 계산
- 점별 잔차와 전체 RMS 출력 → 측정 품질 확인
- 결과: `calib/lekiwi/robot_board.npz` (`theta`, `t`, `R`, `z_board`, …)

### 5) `04.ar_final.py` — 두 창 동시 (포즈 1회 고정)
```bash
python 04.ar_final.py
```
- 좌측 창 `ar_axes` = 카메라 라이브 + 직육면체
- 우측 창 `ar_topdown` = top뷰 라이브 + 격자 교차점마다 로봇 `(x, y)` 라벨
- **처음 `LOCK_FRAMES`(기본 10) 프레임** 동안 보드 포즈를 측정해 평균낸 뒤 **고정**
- 이후 detection/solvePnP를 더 이상 돌리지 않음 — 직육면체 픽셀 좌표, 호모그래피, 격자 라벨 위치 모두 고정
- 좌측은 **라이브 카메라 + 고정 직육면체**, 우측은 **라이브 warp + 고정 라벨**
- 키: `q` 종료, `r` 재측정
- 캔버스 해상도는 파일 상단 `PIXELS_PER_MM = 6` 로 조절

### 6) `05.estimate_object_cord.py` — YOLO 객체 검출 + 로봇 좌표 추정
```bash
python 05.estimate_object_cord.py
```
- 04.ar_final.py 와 동일한 두 창 + 매 프레임 YOLO 추론
- 가중치: 파일 상단 `YOLO_WEIGHTS` 경로 (기본 `~/workspace/yolo/outputs/runs/green_cube_v1/weights/best.pt`)
- 검출된 객체마다:
  - 좌측 카메라 뷰의 **바운딩 박스 중심**에 점 + `클래스 (x, y)` 라벨
  - 우측 top뷰에 `perspectiveTransform`으로 점을 옮기고 **로봇 좌표 (x, y)** 라벨
- HUD에 `det=N` 으로 검출 개수 표시

---

## LeKiwi 라서 생기는 제약

- **베이스가 움직이면 03이 무효가 됩니다.** 카메라도 팔도 베이스에 같이 붙어 있는데,
  보드는 바닥에 따로 있습니다. 베이스가 굴러가면 보드↔로봇 관계(θ, t)가 통째로 바뀌므로
  03을 다시 재야 합니다. **03 측정부터 05 실행까지 베이스를 세워 두세요.**
  (04/05의 `r` 키는 카메라↔보드 포즈만 다시 잡을 뿐, 보드↔로봇은 안 고칩니다.)
- **`wrist` 카메라는 캘리브레이션용으로 쓰면 안 됩니다.** 팔과 함께 움직입니다.
- **팔이 보드를 가리면 검출이 통째로 실패합니다.** `findChessboardCorners` 는 13×9 내부
  코너가 **하나도 안 가려진 채 전부** 보여야 성공합니다 — 부분 검출이란 게 없습니다.
  LeKiwi 는 팔이 front 카메라 시야 한가운데를 지나가므로, 보드를 놓기 전에 팔을
  위/뒤로 접어 시야에서 빼세요. 가려져 있으면 `00.check_link.py` 가
  `chessboard ... seen in 0/N frames` 로 알려줍니다.
- **영상은 JPEG(품질 90)으로 압축되어 옵니다.** 코너 검출에 치명적이진 않지만,
  조명이 어두우면 압축 블록 노이즈로 검출률이 떨어집니다.
- **색 채널이 뒤집혀 옵니다 (R↔B).** lerobot 의 `OpenCVCameraConfig.color_mode` 기본값이
  `RGB` 라서, 카메라가 만든 RGB 배열을 host 가 그대로 `cv2.imencode`(BGR 가정)에
  넣어 보냅니다. [frame_source.py](frame_source.py) 가 `lekiwi.host_color_mode` 를 보고
  되돌려 놓습니다. 이건 보기 문제만이 아니라 **YOLO 정확도에 직접 영향**을 줍니다
  (ultralytics 는 BGR 배열을 기대합니다).
- **프레임률은 host의 `max_loop_freq_hz`(기본 30) 에 묶입니다.** 유선이면 거의
  그대로 나옵니다 (실측 ~28 FPS).

---

## 동작 원리 요약

1. **카메라 캘리브레이션** (1회): `cv2.calibrateCamera` → `K`, `dist`
2. **포즈 추정**: `findChessboardCorners` → `cornerSubPix` → `solvePnP` (rvec, tvec)
3. **보드↔로봇 매핑**: EE 측정 점들로 Procrustes 2D 강체 변환 → θ, t
4. **AR 직육면체**: 보드 내부 영역 위로 8 꼭짓점을 `projectPoints` 로 화면 좌표 변환 → wireframe + 반투명 윗면
5. **Top-down 뷰**: 보드 외곽 4 코너를 화면에 투영 → `getPerspectiveTransform` → `warpPerspective`
6. **격자별 좌표**: 외곽 (i, j) → 보드 mm → 로봇 m
7. **객체 → 로봇 좌표**: YOLO bbox center → `perspectiveTransform` 으로 top뷰 픽셀 → mm → m

---

## `config.yaml` 주요 키
```yaml
board:
  pattern_size: [13, 9]       # 내부 코너
  square_size_mm: 20.0
  axis_length_mm: 60.0
  box_height_mm: 120.0        # 직육면체 높이

colors:                       # BGR
  x: [0, 255, 0]
  y: [255, 0, 0]
  z: [0, 0, 255]

camera:
  source: lekiwi              # local | lekiwi | url
  name: front                 # source=lekiwi 일 때 host가 발행하는 카메라 이름
  index: 4                    # source=local 일 때 장치 인덱스
  width: 1280                 # source=local 전용
  height: 720                 # source=local 전용
  fps: 30                     # source=local 전용
  fourcc: MJPG                # source=local 전용
  url: ""                     # source=url 일 때 MJPEG/RTSP 주소
  rotate: 0                   # 0/90/180/270, 화면이 뒤집혀 보일 때만

lekiwi:
  remote_ip: 10.42.0.61
  port_zmq_observations: 5556
  port_zmq_cmd: 5555
  connect_timeout_s: 10.0     # 첫 프레임 대기
  read_timeout_s: 2.0         # 이후 프레임 하나 대기
  host_color_mode: rgb        # host 가 보내는 채널 순서 (rgb | bgr)

calib:
  file: calib/lekiwi/camera_params.npz
  target_shots: 20

robot_calib:
  file: calib/lekiwi/robot_board.npz
  z_board_m: 0.0804
  touch_points:
    - {i, j, x, y, z}   # 외곽 격자 인덱스 i=0..14, j=0..10 + 로봇 좌표(m)

display:
  label_stride: 1
  label_unit: m               # "m" | "mm"
  label_decimals: 3
  label_font_scale: 0.35
```

`camera.source` 를 `local` 로 두면 main 브랜치와 똑같이 노트북 웹캠으로 동작합니다.

---

## 파일 구조
```
Cal_checkerboard_yolo/
├── README.md
├── requirements.txt
├── config.yaml
├── frame_source.py            # local / lekiwi / url 프레임 소스
├── 00.check_link.py           # 연결 점검
├── 01.calibrate.py
├── 02.ar_axes.py
├── 03.robot_calib.py
├── 04.ar_final.py
├── 05.estimate_object_cord.py
└── calib/
    └── lekiwi/
        ├── camera_params.npz  # 01 결과
        └── robot_board.npz    # 03 결과
```

---

## 문제 해결

**`No observation from the LeKiwi host at tcp://10.42.0.61:5556`**
- 파이에서 host가 안 떠 있습니다. 위 [0단계](#0-라즈베리파이에서-host-띄우기-매-수업-시작마다) 참고.
- `--host.connection_time_s` 를 안 줘서 30초 만에 꺼졌을 수 있습니다.
- `ping 10.42.0.61` 이 되는지, 랜선이 빠지지 않았는지 확인.

**프레임이 뚝뚝 끊기거나 FPS가 절반**
- 다른 클라이언트(앱 / teleop / record)가 같은 스트림을 같이 빨고 있습니다. 하나만 켜세요.

**`Camera 'front' is not in the stream`**
- host가 그 이름으로 카메라를 발행하지 않습니다. 에러 메시지에 실제 이름 목록이 찍히니
  `config.yaml` 의 `camera.name` 을 거기 맞추세요.

**`front` 로 골랐는데 손목에서 본 화면이 나옴 (킷마다 뒤바뀜)**
- 이름은 host 설정(`--robot.cameras=...`)이 붙인 라벨일 뿐이라 실제 장치와 다를 수 있습니다.
- `00.check_link.py` 라이브 창에서 `n` 으로 전환해 어느 쪽이 베이스 고정인지 눈으로 확인하고,
  `config.yaml` 의 `camera.name` 을 그쪽으로 바꾸세요.
- 파이 쪽을 바로잡으려면 host 를 띄울 때 `--robot.cameras=` 의 `index_or_path` 를 서로 바꿉니다.

**`chessboard 13x9 seen in 0/N frames` (연결은 되는데 보드를 못 찾음)**
- 팔이 보드를 가리고 있지 않은지 먼저 보세요. 가장 흔한 원인입니다.
- 보드 전체가 화면 안에 들어오고, 바깥쪽에 흰 여백이 남아 있어야 합니다.
- 보드가 너무 작게(멀리) 잡히면 코너를 못 씁니다. 640×480 에서는 보드가 화면 폭의
  절반 이상을 차지하게 두세요.
- `board.pattern_size` 가 실제 보드와 맞는지 확인 (내부 코너 개수이지 칸 수가 아닙니다).
- 조명 반사로 흰 칸이 날아가면 검출이 흔들립니다.

**포즈는 잡히는데 로봇 좌표가 엉뚱함**
- `03.robot_calib.py` 의 RMS를 보세요. 4~5mm면 정상, 수 cm면 측정이 틀린 겁니다.
- touch_points가 한 줄로 몰려 있으면 해가 불안정합니다.
- 03 이후에 베이스를 움직였다면 다시 재야 합니다.

**`Missing YOLO weights: ...best.pt`**
- `05.estimate_object_cord.py` 상단 `YOLO_WEIGHTS` 를 실제 학습된 가중치 경로로 고치세요.

**색이 이상함 — 보라색이 분홍으로, 노란색이 파랑으로 보임**
- R 과 B 채널이 뒤바뀐 것입니다. `lekiwi.host_color_mode` 를 `rgb` 로 두세요(기본값).
- 반대로 host 를 `--robot.cameras=...color_mode: bgr...` 로 띄웠다면 `bgr` 로 바꾸세요.

**화면이 180° 뒤집혀 보임**
- host의 카메라 `rotation` 설정 탓입니다. 파이 쪽을 고치는 게 정석이고,
  급하면 `config.yaml` 의 `camera.rotate` 로 클라이언트에서 돌릴 수 있습니다.
  (단, 01 캘리브레이션도 같은 `rotate` 값으로 다시 해야 합니다.)

---

## 주의 / 가정
- **조명**: 코너 검출 안정성을 위해 충분한 균일 조명이 필요합니다.
- **광각 왜곡**: 캘리브레이션 품질이 결과 정확도에 직결됩니다.
- **보드 평면성**: 보드가 바닥에 평평히 놓여 있어야 z=상수 가정이 성립합니다.
- **z축 정렬**: 로봇 z축과 보드 법선이 평행하다고 가정. 바닥이 비스듬하면 오차 증가.
- **EE 측정 오차**: 4~5 mm 수준의 잔차는 흔합니다 (`03.robot_calib.py` 의 RMS 출력으로 확인).
- **YOLO 가중치**: 05 실행 전 `YOLO_WEIGHTS` 경로의 모델이 존재해야 합니다.
