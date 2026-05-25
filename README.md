# Checkerboard ↔ Robot Coordinate Mapping & AR

웹캠으로 인쇄 체커보드를 잡아 **로봇 베이스 좌표계와 매핑**하고, 그 위에서
- 보드 위에 3D 직육면체 AR 오버레이를 띄우고
- 위에서 본 평면 뷰(top-down)에서 격자 교차점마다 로봇 좌표를 표기하고
- YOLO로 검출한 객체의 바운딩 박스 중심점을 로봇 좌표로 환산해 보여주는

데모입니다. 모든 설정은 [config.yaml](config.yaml)에서 관리됩니다.

---

## 보드 사양
- A4 인쇄, 정사각형 한 변 **20mm**, 14×10 squares (= **13×9 내부 코너**, 15×11 외곽 교차점)
- 보드는 가로(긴 변) > 세로(짧은 변)로 두고 고정 사용
- 인쇄는 **실제 크기 100%** 로 출력 후, 자로 한 칸을 측정해 20mm 인지 확인 (다르면 `config.yaml`의 `square_size_mm` 보정)

좌표계 정의:
- **보드 프레임 B**: 원점 = 보드 좌하단 외곽 코너, +X = 가로(긴 변), +Y = 세로(짧은 변), z=0 보드면
- **로봇 프레임 R**: 사용자 로봇 베이스
- z축은 두 프레임이 평행하다고 가정 (보드를 책상 위에 평평히 둠)

---

## 설치
```bash
pip install -r requirements.txt
```
- `opencv-python`, `numpy`, `PyYAML`, `ultralytics` (YOLO)

---

## 실행 순서 (01 → 07)

### 1) `01.calibrate.py` — 카메라 내부 파라미터 캘리브레이션 (1회)
```bash
python 01.calibrate.py
```
- 체커보드를 다양한 각도/거리에서 비추며 **SPACE** 로 캡처
- 키: `SPACE`=캡처, `r`=초기화, `ESC`=종료
- 목표 매수(`target_shots`, 기본 20장) 도달 시 `cv2.calibrateCamera` 실행
- RMS 재투영 오차가 < 1.0px 이면 양호
- 결과: `calib/camera_params.npz` (`K`, `dist`, `rms`, `image_size`)

### 2) `02.ar_axes.py` — 보드 위 직육면체 AR (검증용)
```bash
python 02.ar_axes.py
```
- `calib/camera_params.npz` 를 로드해 매 프레임 `findChessboardCorners → solvePnP`
- 보드 내부 코너 영역(12×8 격자) 위에 높이 `box_height_mm` 직육면체를 그림
- 색: 밑면=초록(X), 기둥=파랑(Y), 윗면=빨강(Z)
- `q` 종료

### 3) EE로 보드 코너 4점 측정 → `config.yaml` 직접 편집
EE(엔드 이펙터)로 보드 위 정해진 외곽 격자점을 찍어 로봇 베이스 좌표를 기록합니다.
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

좋은 측정 가이드:
- X(긴 변) 방향과 Y(짧은 변) 방향 모두에서 충분한 베이스라인을 확보하도록 분포
- 최소 2점, 3점 이상 권장 (잔차 RMS 산출용)

### 4) `03.robot_calib.py` — 보드 ↔ 로봇 2D 강체 변환 추정
```bash
python 03.robot_calib.py        # 풀고 저장
python 03.robot_calib.py show   # 풀어보기만 (저장 X)
```
- yaml의 `touch_points` 로부터 Procrustes(SVD)로 (θ, t) 계산
- 점별 잔차와 전체 RMS 출력 → 측정 품질 확인
- 결과: `calib/robot_board.npz` (`theta`, `t`, `R`, `z_board`, …)

### 5) `04.ar_grid.py` — 카메라 원근 뷰 + 격자별 로봇 좌표 라벨
```bash
python 04.ar_grid.py
```
- 카메라 영상에 15×11 외곽 격자 교차점 모두에 점 + `(x, y)` 라벨 표시
- 라벨 단위/소수점/글자 크기/표시 간격은 `config.yaml`의 `display.*` 로 조절

### 6) `05.ar_topdown.py` — 보드를 위에서 본 평면 뷰
```bash
python 05.ar_topdown.py
```
- `getPerspectiveTransform` + `warpPerspective` 로 보드 영역만 직사각형으로 펴서 표시
- 캔버스 해상도는 파일 상단 `PIXELS_PER_MM = 6` 로 조절 (1680×1200 px)

### 7) `06.ar_final.py` — 두 창 동시 (포즈 1회 고정)
```bash
python 06.ar_final.py
```
- 좌측 창 `ar_axes` = 카메라 라이브 + 직육면체
- 우측 창 `ar_topdown` = top뷰 라이브 + 격자 좌표 라벨
- **처음 `LOCK_FRAMES`(기본 10)** 프레임 동안 보드 포즈를 측정해 평균낸 뒤 **고정**
- 이후로는 detection/solvePnP를 더 이상 돌리지 않음 — 직육면체와 라벨 위치, 호모그래피 모두 고정
- 좌측은 라이브 카메라 위에 고정 직육면체, 우측은 라이브 warp + 고정 라벨
- 키: `q` 종료, `r` 재측정

### 8) `07.estimate_object_cord.py` — YOLO 객체 검출 + 로봇 좌표 추정
```bash
python 07.estimate_object_cord.py
```
- 06.ar_final.py 와 동일한 두 창 + 매 프레임 YOLO 추론
- 가중치: 파일 상단 `YOLO_WEIGHTS` 경로 (기본 `~/workspace/yolo/outputs/runs/green_cube_v1/weights/best.pt`)
- 검출된 객체마다:
  - 좌측 카메라 뷰의 **바운딩 박스 중심**에 점 + `클래스 (x, y)` 라벨
  - 우측 top뷰에 `perspectiveTransform`으로 점을 옮기고 **로봇 좌표 (x, y)** 라벨
- HUD에 `det=N` 으로 검출 개수 표시

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
  index: 4
  width: 1280
  height: 720
  fps: 30
  fourcc: MJPG

calib:
  file: calib/camera_params.npz
  target_shots: 20

robot_calib:
  file: calib/robot_board.npz
  z_board_m: 0.0804
  touch_points:
    - {i, j, x, y, z}   # 외곽 격자 인덱스 i=0..14, j=0..10 + 로봇 좌표(m)

display:
  label_stride: 1
  label_unit: m               # "m" | "mm"
  label_decimals: 3
  label_font_scale: 0.35
```

---

## 파일 구조
```
checkerboard/
├── README.md
├── requirements.txt
├── config.yaml
├── 01.calibrate.py
├── 02.ar_axes.py
├── 03.robot_calib.py
├── 04.ar_grid.py
├── 05.ar_topdown.py
├── 06.ar_final.py
├── 07.estimate_object_cord.py
└── calib/
    ├── camera_params.npz      # 01 결과
    └── robot_board.npz        # 03 결과
```

---

## 주의 / 가정
- **조명**: 코너 검출 안정성을 위해 충분한 균일 조명이 필요합니다.
- **광각 왜곡**: 노트북 내장 웹캠은 왜곡이 크므로 캘리브레이션 품질이 결과 정확도에 직결됩니다.
- **보드 평면성**: 보드가 책상 위에 평평히 놓여 있어야 z=상수 가정이 성립합니다.
- **z축 정렬**: 로봇 z축과 보드 법선이 평행하다고 가정. 책상이 비스듬하면 오차 증가.
- **EE 측정 오차**: 4~5 mm 수준의 잔차는 흔합니다 (`03.robot_calib.py` 의 RMS 출력으로 확인).
- **YOLO 가중치**: 07 실행 전 `YOLO_WEIGHTS` 경로의 모델이 존재해야 합니다.
