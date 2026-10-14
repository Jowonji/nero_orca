# 네이티브 Linux(듀얼부팅)에서 Nero+Orca LeRobot 데이터 수집 환경 구성 기록

| 항목 | 내용 |
| --- | --- |
| 날짜 | 2026-10-14 |
| 범위 | Dell 노트북 Ubuntu 듀얼부팅에서 실물 Nero 팔 + Orca 손 + Orbbec 2대로 `record_dataset.py` 녹화 (shadow 검증 → real 단계 시작) |
| 환경 | Ubuntu 24.04, 커널 7.0, X11, Intel Iris Xe, Python 3.12 (`orca_teleop/.venv`, uv), lerobot 0.4.4, torch 2.10 |
| 선행 | [wsl-lerobot-recording-setup.md](wsl-lerobot-recording-setup.md), [real-robot-bringup.md](real-robot-bringup.md), [jetson-webcam-teleop-first-run.md](jetson-webcam-teleop-first-run.md) |
| 결과 | Orbbec 깊이 모드, 좌우 반전, pick-and-place 3자유도 모드. 손 모터 방향 수정 후 팔·손 real 96초 에피소드: 팔 오차 평균 0.2~0.4°, 손 5°, 안전장치 0회 |

---

## 1. 요약

- WSL에서 막혔던 CAN 끊김과 usbipd 문제는 네이티브 Linux에서 사라졌다. 장치는 꽂으면 바로 잡힌다.
- 설치 중 패키지 충돌 2건(`av`, OpenCV)을 정리했다 (§2.3).
- 녹화 데이터 문제 3개를 고쳤다 (§4).
  - Orca 손 캘리브레이션이 어긋나 값이 한계로 잘림 → 재캘리브레이션
  - `observation.state`가 MuJoCo 모델값이라 손 MCP가 -24.4°에서 잘림 → 하드웨어 모드는 센서값 저장
  - 녹화가 15 Hz를 못 채움 (10초에 75~121프레임) → retargeter 속도 제한
- 화면 배치: 웹캠 창은 왼쪽 위, MuJoCo 창은 오른쪽에 사이드 패널 없이, 로봇에 맞춘 시점으로 뜬다 (§5).
- Orbbec 깊이 모드를 살렸다 (시리얼 지정 + 레이저 3단계, §8). 팔 이동은 손 1 cm → 1.5 cm, 좌우 반전 옵션 (§9.1).
- `--pick-place`: 손목 쪽 관절을 고정한 3자유도 팔 (§9.2). 팔만 real 40초 에피소드에서 손목 위치 오차 평균 0.4 cm, 안전장치 0회 (§10).
- CAN 어댑터가 반복해서 끊기고, CAN 선이 빠졌을 때 팔이 쓰러졌다. 이슈 목록은 §11.
- **Orca 손가락이 실물에서 반대로 움직이고 있었다** (Feetech 모터 방향). 설정 부호를 뒤집고 재캘리브레이션 (§11). 팔·손 real 96초 녹화 확인 (§10). 수집은 `record_pick.sh` (§10), 카메라 배치는 §12.

---

## 2. 설치

### 2.1 시스템 패키지와 권한

```bash
sudo apt install -y can-utils v4l-utils libsm6 libice6 ffmpeg wmctrl
sudo usermod -aG dialout $USER     # 손 시리얼 권한. 로그아웃 후 다시 로그인해야 적용
```

- `ffmpeg`: 없으면 녹화는 되지만 `LeRobotDataset`이 영상을 읽지 못한다 (torchcodec이 `libavutil` 못 찾음).
- `wmctrl`: MuJoCo 창 자동 배치용. 없어도 녹화는 된다.

### 2.2 Python 환경

WSL 때와 같이 `orca_teleop/.venv`에 uv로 설치한다. `uv.lock`으로 버전이 고정된다. conda env에 pip로 설치해도 되지만 버전 고정이 안 된다.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
cd ~/workspace/orca_teleop
uv sync --python 3.12 --extra learning --extra sim
uv pip install pyorbbecsdk2 python-can "git+https://github.com/agilexrobotics/pyAgxArm.git"
```

Orbbec udev 규칙 (긴 명령은 붙여 넣을 때 줄이 깨지므로 나눠서 실행):

```bash
cd ~/workspace/orca_teleop/.venv/lib/python3.12/site-packages/pyorbbecsdk/shared
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2.3 패키지 충돌 (설치 후 꼭 확인)

| 문제 | 원인 | 해결 |
| --- | --- | --- |
| `av`가 12.3.0으로 내려감 | `pyorbbecsdk2`가 `av==12.3.0` 고정. lerobot은 `>=15` 필요 | `uv pip install "av==15.1.0"`. pyorbbecsdk는 예제 파일에서만 `av`를 쓴다 |
| 웹캠 미리보기 창이 안 뜨고 손 추적 프로세스가 죽음 (`cvNamedWindow ... not implemented`) | `opencv-python`, `opencv-contrib-python`, `opencv-python-headless`(lerobot 의존)가 같은 `cv2` 폴더를 덮어씀. headless가 마지막이면 GUI 없음 | `uv pip install --reinstall-package opencv-contrib-python "opencv-contrib-python==4.13.0.92"` |

`uv sync`나 `uv pip install pyorbbecsdk2`를 다시 실행하면 둘 다 재발할 수 있다. 확인:

```bash
.venv/bin/python -c "import av, cv2; print(av.__version__); print([l for l in cv2.getBuildInformation().splitlines() if 'GUI' in l])"
# 15.1.0 / GUI: QT5 이어야 함
```

---

## 3. 장치 연결

| 장치 | Linux 장치 | 녹화 인자 |
| --- | --- | --- |
| Nero CAN 어댑터 (candleLight `1d50:606f`) | `can0` (gs_usb) | `--nero-can can0` (기본) |
| Orca 손 (CH343 `1a86:55d3`) | `/dev/ttyACM0`, by-id `usb-1a86_USB_Single_Serial_5B79029607-if00` | `--model-path config/orcahand_right/config.yaml` |
| Orbbec 데이터 수집용 (`CP0H953001AR`) | 이번 연결에서 `/dev/video18` | `--camera data:18` |
| Orbbec 손 추적용 (`CP0H95300129`) | 이번 연결에서 `/dev/video10` | `--teleop-camera 10` |

- 손 설정의 by-id 경로는 WSL과 같아서 그대로 쓴다.
- `/dev/video` 번호는 다시 꽂으면 바뀐다. 녹화 전에 확인:

```bash
for i in $(ls /dev/video* | sed 's#/dev/video##' | sort -n); do
  v4l2-ctl -d /dev/video$i --list-formats 2>/dev/null | grep -q MJPG && \
  echo "video$i $(udevadm info -q property -n /dev/video$i | grep ID_SERIAL_SHORT | cut -d= -f2)"
done
```

- **CAN 어댑터가 허브의 특정 포트에서 인식되지 않았다** (WSL 때와 같은 증상). 다른 포트로 옮겨 해결. `lsusb | grep 1d50`으로 확인한다.

### 3.1 CAN

```bash
sudo ip link set can0 up type can bitrate 1000000
```

- 웹 컨트롤러(192.168.0.150)에서 **External CAN Push**를 눌러야 피드백이 온다. 정상은 초당 약 2100개 (`0x251~0x257`, `0x261~0x267`, `0x2A1~0x2A9`).
- 1분쯤 들어오다가 0개로 멈춘 일이 한 번 있었다. External CAN Push를 다시 누르니 복구됐다. 어댑터나 버스 문제가 아니었다 (버스 에러 0).
- 수신 속도 확인 (`candump`를 `timeout`으로 끊으면 출력 버퍼가 버려져 0개로 보이니 카운터로 잰다):

```bash
a=$(cat /sys/class/net/can0/statistics/rx_packets); sleep 1; echo $(( $(cat /sys/class/net/can0/statistics/rx_packets) - a ))
```

- Nero 관절값이 모두 0 근처로 읽힌 것은 팔이 실제로 0° 자세(곧게 선 자세)였기 때문이다. 0.001~0.006° 노이즈가 있어 실제 엔코더 값임을 확인했다.

---

## 4. 녹화 데이터 문제와 수정

### 4.1 Orca 손 캘리브레이션 재실행

- 증상: 손 17관절 값이 녹화 내내 같은 값(106.5°, -24.5° 등)으로 고정. 종료 때 모터 12개 `Motor ID N is out of bounds`.
- 원인: 10/12 WSL 캘리브레이션의 모터 범위와 지금 모터 위치가 맞지 않음. 범위 밖이라 관절값이 한계로 잘렸다.
- 조치: [wsl-lerobot-recording-setup.md](wsl-lerobot-recording-setup.md) §5의 명령으로 재캘리브레이션 (손목은 기존 값 유지). `config/orcahand_right/calibration.yaml` 갱신. 이후 out of bounds 경고 없음.

### 4.2 `observation.state`를 하드웨어 센서값으로 (`combined_sink.py`)

- 증상: 재캘리브레이션 후에도 다섯 손가락 MCP가 모두 정확히 -24.4°. 직접 읽은 값은 -35~-49°.
- 원인: `_read_joint_state()`가 MuJoCo `qpos`를 저장했다. 손은 실물 값을 `ctrl` 목표로만 받고, 물리 시뮬레이션이 MJCF 관절 한계(MCP -24.4°)에서 멈춘다. 실물 손 범위는 -60°까지다.
- 수정: 하드웨어 모드에서는 Nero `q`와 Orca `deg`(장치 피드백)를 그대로 저장한다. sim 모드는 그대로 `qpos`.
- 결과: MCP -34.9 / -46.9 / -48.6° 등 실제 값으로 저장.

### 4.3 녹화 프레임 누락 (`orca_teleop`: `pipeline.py`, `record_dataset.py`)

| 녹화 | 10초 프레임 |
| --- | --- |
| 첫 shadow | 75 |
| 이후 shadow | 121, 101 |
| `--stub` (손 추적 없음) | 150 |
| 수정 후 shadow | 150, 150 |

- 하드웨어 읽기는 빠르다: `get_observation()` 6 ms, 손 sync read 5.5 ms, MuJoCo 렌더 9.5 ms.
- 원인: retarget 1회가 약 31 ms의 Python 연산이다 (torch 스레드 수 14/4/1에서 32.7/30.7/30.2 ms로 거의 같음). 웹캠 프레임마다(초당 25~30회) 돌면서 GIL을 거의 계속 잡아, 같은 프로세스의 15 Hz 녹화 루프가 틱을 놓쳤다. 이때 데이터셋은 15 fps로 기록되므로 재생 속도도 틀어진다.
- 수정: `retargeter_worker(max_hz=...)` 추가. 지정하면 그 속도 이하로만 돌고 큐에 쌓인 프레임 중 최신 것만 계산한다. `record_dataset.py`가 `--fps`를 넘긴다. 기본값은 꺼져 있어 다른 스크립트(데이터셋 재생 등)는 모든 프레임을 그대로 처리한다.

---

## 5. 화면 배치

- 웹캠 랜드마크 창: 왼쪽 위 640×480 (`orca_teleop/.../mediapipe/publisher.py`).
- MuJoCo 창: 웹캠 창 오른쪽(x=680)부터 화면 끝까지. `launch_passive`에 위치 옵션이 없어 창이 뜬 뒤 `wmctrl`로 옮긴다 (`_place_viewer_right()`). Wayland나 wmctrl이 없으면 원래 자리에 뜬다.
- MuJoCo 사이드 패널은 기본으로 숨김. `Tab` / `Shift+Tab`으로 연다.
- 뷰어 시점은 열릴 때 로봇 현재 자세의 바운딩 박스에 맞춘다 (`_frame_robot()`). 녹화되는 `frontal` 카메라는 에피소드 간 구도 유지를 위해 고정값 그대로다.

---

## 6. 녹화 명령

shadow (읽기만, 명령 안 보냄. 단 `orca_core` 연결 시 손 토크는 켜짐):

```bash
cd ~/workspace/nero_orca
../orca_teleop/.venv/bin/python record_dataset.py \
  --local --source mediapipe --show-video \
  --depth webcam --teleop-camera 10 --camera data:18 \
  --nero-output shadow --orca-output shadow \
  --model-path config/orcahand_right/config.yaml \
  --urdf-path ../orcahand_description/v1/models/urdf/orcahand_right.urdf \
  --repo-id local/nero-orca-shadow-test \
  --root datasets/nero-orca-shadow-test \
  --task "shadow test" \
  --fps 15 --num-episodes 1 --episode-seconds 10 \
  --overwrite 2>&1 | tee record_test.log
```

- 손 추적 카메라에 오른손이 보여야 녹화가 시작된다 (`teleop=0/5`에서 멈추면 손 추적 문제).
- 타이머 모드는 끝날 때까지 기다린다. 중간에 Ctrl+C를 누르면 마무리 저장이 안 될 수 있다.
- `--depth auto` (Orbbec 깊이)는 네이티브 Linux에서 아직 시험하지 않았다.

저장 확인:

```bash
../orca_teleop/.venv/bin/python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('local/nero-orca-shadow-test', root='datasets/nero-orca-shadow-test')
print(ds.num_episodes, ds.num_frames, ds[0]['observation.images.data'].shape)"
```

---

## 7. real 모드 단계

관절 1~7번 방향은 2026-09-18 Jetson에서 실물로 확인했다 (모두 같은 방향, `NERO_JOINT_SIGN`/`OFFSET` 기본값 유지, [jetson-webcam-teleop-first-run.md](jetson-webcam-teleop-first-run.md)). 그 뒤 팔·모델을 바꾼 일이 있으면 `mirror_real.py --orca sim`으로 다시 확인한다.

공통:

- 주변을 비우고 비상정지를 손 닿는 곳에 둔다. 녹화 중 웹 페이지를 조작하지 않는다.
- real로 켜면 **팔이 바로 대기 자세(`REST_QPOS`, 4번 70°, 7번 75°)로 이동한다.** 0° 자세에서 시작하면 팔꿈치가 크게 굽혀진다.
- 종료 후에도 Nero는 enable 상태로 남는다. 팔을 받친 뒤 웹에서 disable한다.
- 이상하면 비상정지 → Ctrl+C.

| 단계 | 바꿀 인자 | 볼 것 |
| --- | --- | --- |
| 1. 팔만 real, 최저속 | `--nero-output real --nero-speed 10 --orca-output sim --episode-end space` | 대기 자세까지 부드럽게 가는지, 손을 작게 움직이면 같은 방향으로 따라오는지 |
| 2. 손도 real | `--orca-output real` | 손가락을 천천히 쥐었다 폈을 때 실물 손이 따라오는지 |
| 3. 수집 | `--task`, `--num-episodes`, `--repo-id`, `--root`. 속도는 10에서 20까지 천천히 | 이어 붙일 때는 `--overwrite`를 뺀다 |

- `--episode-end space`: **녹화를 실행한 터미널을 클릭한 뒤** 스페이스바로 저장하고 다음 에피소드, `q`/Esc로 저장 후 종료. 웹캠 창이 활성화된 상태에서 `q`를 누르면 손 추적 프로세스만 꺼지고 녹화는 저장되지 않는다 (§11).
- 실제로 진행한 단계와 결과는 §10.

---

## 8. Orbbec 깊이 모드

- `--depth webcam`은 Orbbec을 일반 웹캠으로만 쓴다. 팔 이동량을 화면 비율로 계산해서 카메라를 멀리 둘수록 좁게 움직인다.
- `--depth orbbec`은 손바닥의 실제 3D 위치(m)를 쓴다. 손 1 cm → 팔 1.5 cm (`METRIC_SCALE` 1.5), 카메라 거리와 무관하다.

필요한 인자:

| 인자 | 이유 |
| --- | --- |
| `--teleop-orbbec-serial CP0H95300129` | Orbbec이 2대라 기본 `Pipeline()`은 첫 번째 장치를 연다 (녹화용 카메라일 수 있음). 시리얼을 주면 SDK가 V4L2 백엔드를 쓴다. 기본 libuvc 백엔드는 uvcvideo를 떼어 가서 다른 Orbbec의 `/dev/video` 색상 노드가 사라졌다 (재연결로 복구) |
| `--teleop-orbbec-laser-level 3` | 색상+깊이에 IR 레이저 5~6단계면 카메라가 0.1초 만에 USB에서 빠졌다 (버스 전원 허브의 전력 부족 추정. 깊이만, 또는 레이저 끔은 정상). 1~4단계 정상, 중앙 깊이 유효 92~94%로 차이 없음 |

검증: 데이터 카메라를 OpenCV로 동시에 읽으면서 15초 — RGB-D 28.9 fps(중앙 유효 91%), 데이터 카메라 29.1 fps, 리셋 없음.

---

## 9. 팔 이동 매핑 변경

### 9.1 범위와 좌우 반전

- `METRIC_SCALE` 1.0 → 1.5, `POS_CLAMP` ±18/25/20 → ±25/30/25 cm (모델 중력 부하 최대 76%, 팔을 멀리 뻗고 올린 모서리만 미도달).
- `--mirror-lateral`: 손 추적 카메라가 조작자를 마주 보면 거울상이라 좌우가 반대다. 로봇 Y 부호만 뒤집는다.
- **시작 기준점**: 손이 처음 인식된 위치가 시작 자세에 대응한다. 아래에서 손을 들어 올리며 화면에 넣으면 그 낮은 위치가 기준이 되어 더 내려가지 못한다. 손을 높게 들고 시작하거나, 손을 0.5초 이상 화면 밖으로 뺐다가(팔 정지) 다시 보여 주면 그 위치가 새 기준점이 된다 (클러치).

### 9.2 pick-and-place 모드 (`--pick-place`)

학습할 action을 단순하게 하려고 손 방향을 고정한 3자유도 팔로 쓴다 (`arm_ik.py`의 `PICK_*`).

| 항목 | 값 |
| --- | --- |
| 고정 관절 | J3(위팔 비틀기), J5(아래팔 비틀기), J6(손목 좌우) = 0. 이때 J2, J4, J7 축이 평행해진다 |
| 움직이는 관절 | J1, J2, J4로 손 위치. J7 = 135° − J2 − J4로 손 기울기 고정 (손끝 약 45° 아래) |
| 시작 자세 | 로봇 앞 52 cm, 손바닥 중심(carpals) 높이 23 cm. 관절 J2 24°, J4 80°, J7 32° |
| 범위 (시작 자세 기준) | 앞뒤 ±10 cm, 좌우 ±25 cm, 아래 −9 / 위 +11 cm. 책상 = 로봇 받침 면. 손 최저점이 carpals보다 펴진 손 9.8 cm, 쥔 손 11.9 cm 아래라 맨 아래에서도 펴진 손 4.2 cm, 쥔 손 2 cm 이상 책상 위 |
| 도달 거리 | 목표의 수평 거리를 높이별 도달 한계 안(1.5 cm 여유)으로 당김: carpals 높이 14 cm에서 67.5 cm, 34 cm에서 59 cm, 최소 40 cm. 닿지 않는 먼 모서리를 겨냥하면 IK가 가장 가까운 자세로 가면서 손이 책상 0.6 cm 위까지 처졌다 |
| Orca 손목 | 0° 고정 (`PICK_ORCA_WRIST_DEG`). IK는 항상 이 각도로 손 위치를 계산한다 (EE body가 Orca 손목 뒤에 있어 손목 각도가 EE를 옮긴다) |
| 팔꿈치 | J4 ≥ 15°, J2+J4는 J7이 한계 안에 있도록 제한 |
| 속도 | 손목 목표 0.3 m/s, 관절 명령 0.4 rad/s (전 관절 같은 비율로 줄여 J7 관계 유지). `--nero-speed 20` |

- 손 기울기를 고정하면 닿는 범위가 좁아진다. 대기 자세(`REST_QPOS`) 기울기로 고정하면 앞뒤 28 × 높이 16 cm뿐이라, 기울기와 시작 자세를 새로 정했다. 기울기별 범위: 90°(수평) 앞 0~50 × 높이 42~64 cm, 135° 앞 42~62 × 높이 0~34 cm, 150° 앞 38~60 × 높이 0~28 cm.
- J1이 돌면 손도 같이 좌우를 향한다.
- `go_home()`(2번째 에피소드부터)과 시작 시 `move_home`도 이 시작 자세로 간다. `limits.apply_rest_pose()`가 자세를 인자로 받는다.
- 검증: 범위 안 5×5×5 = 125개 목표 모두 오차 0.01 cm 이하, 팔꿈치 ≥ 26°, J7 6~67°.
- 시도했다가 되돌린 손 각도 (물체를 위에서 감싸 쥐기 어렵다는 피드백 후):
  - Orca 손목 +30°: 손끝 75° 아래. 너무 꺾였다는 피드백.
  - 손바닥 수평: Orca 손목 −44.5°, 또는 팔 기울기 90.5°(손 쪽 J7 약 −40°). 팔 기울기로 하면 로봇에서 47 cm보다 가까운 곳은 낮게 못 내려가 시작 자세를 56 cm로 옮겨야 했다.
  - 결국 원래 135°(손끝 45° 아래)로 돌아왔다. 다시 바꿀 때는 그 기울기로 손 최저점과 도달 거리를 다시 계산해야 한다.

---

## 10. real 모드 결과 (깊이 모드)

| 실행 | 설정 | 결과 |
| --- | --- | --- |
| 14:42 | 자유 모드, 속도 10%, 관절 0.6 rad/s | 추종 오차 안전장치 2회. 빠른 동작에서 J2가 명령(최대 76°/s)을 못 따라가 최대 41° 차이 |
| 15:11 | pick-place 첫 버전 | **팔꿈치가 반대로 꺾임** (J4 −24°, J7 89° 한계). 범위 끝에서 IK가 J4를 0° 너머로 넘김. 안전장치 12회 → J4 ≥ 15° 제한 추가 |
| 15:15 | 팔꿈치 제한, 속도 10%, 0.6 rad/s | 꺾임 해결 (J4 최소 32°). 안전장치 5회: 명령 J1 33°/s를 실물이 17°/s로만 따라감 → 0.4 rad/s, 속도 20% |
| 15:18 | 속도 20%, 0.4 rad/s | 끊기기 전 22초: 관절 오차 평균 0.3~0.5°, 손목 0.5 cm. CAN 어댑터 끊김으로 이후 무효 (§11) |
| **15:22** | 같은 설정, CAN 포트 변경 | **40.5초, 안전장치 0회. 관절 오차 평균 0.2~0.4°(최대 4.1°), 손목 위치 평균 0.4 cm, 상위 5% 1.2 cm, 최대 4.6 cm. 지연 133 ms.** 범위 앞뒤 ±11, 좌우 −22~+24, 상하 ±11 cm 사용, J4 최소 15° |
| 16:02 | 팔 shadow, **손 real** | 센서상 명령을 따라감 (상관 0.82~0.97, 지연 333 ms, 응답 누락 0). 그러나 **실물 손가락은 반대로 움직이고 있었다** (§11, 손 방향) |
| **16:38** | 팔·손 real, 손 방향 수정 후 | **96초, 안전장치 0회. 팔 관절 오차 평균 0.2~0.4°, 손 평균 5.0°(상위 5% 18.8°, 지연·물체 접촉).** 주먹이 실물 손에도 반영됨 |

수집은 `record_pick.sh`로 한다. 긴 명령을 붙여 넣다 줄이 깨져 `--task: command not found`로 실패한 일이 있어서 스크립트로 묶었다. 데이터 카메라를 시리얼로 찾아 `/dev/video` 번호 변화(18 → 12 → 14 → 10)를 신경 쓰지 않아도 되고, `can0`이 꺼져 있으면 시작 전에 멈춘다.

```bash
cd ~/workspace/nero_orca
./record_pick.sh                                   # 팔·손 real, 속도 20%, 1 에피소드, 덮어쓰기
NERO=shadow ORCA=shadow NAME=hand-view-a ./record_pick.sh
NAME=pick-tape TASK="pick up the tape and place it on the table" EPISODES=10 KEEP=1 ./record_pick.sh
```

- 저장 위치: `datasets/<NAME>/` (`data/` parquet, `videos/` mp4, `meta/`). 기본 이름 `nero-orca-pick-real-test`는 매번 덮어쓴다.
- `KEEP=1`은 같은 데이터셋에 에피소드를 이어 붙인다 (`--overwrite` 없음). 새 이름의 첫 실행은 `KEEP=1` 없이.
- 고정 옵션: `--depth orbbec`, 손 추적 Orbbec 시리얼과 레이저 3단계, `--mirror-lateral --pick-place`, 15 fps, `--episode-end space`.

---

## 11. 이슈 기록

| 이슈 | 증상 | 원인 / 조치 |
| --- | --- | --- |
| **CAN 어댑터 반복 끊김** | 오늘 6회 이상 USB 연결 해제. 15:18에는 녹화 중 `failed to xmit URB: -EPIPE` 후 연결 해제 → `stale Nero joint feedback`로 명령 정지, 에피소드 뒷부분 무효 | 허브 USB 2.0 쪽 포트 `3-3.1.3.1`에서 반복. `3-3.1.3.4`로 옮긴 뒤 15:22 녹화는 끊김 없음. 케이블 교체, 전원 공급 허브 검토 필요 |
| **CAN 선이 빠지자 팔이 쓰러짐** | enable(CAN 제어) 상태에서 선 정리 중 CAN 선을 뽑자 팔이 힘이 빠져 쓰러짐 | 컨트롤러가 통신 끊김에 토크를 끈 것으로 추정. 선 작업 전에 팔을 받치거나 책상에 손을 얹혀 둔 뒤 웹에서 disable. 복구: 손을 책상에 얹혀 받친 상태에서 CAN 재연결 → real 실행의 `move_home`(경로상 손이 책상에서 멀어지는지 모델로 확인) |
| CAN 재연결 후 `can0` DOWN | 어댑터를 다시 꽂으면 `Device 'can0' does not exist` 또는 DOWN | USB가 새로 잡힐 때마다 `sudo ip link set can0 up type can bitrate 1000000` 다시 실행 + External CAN Push. 어댑터를 꽂은 **뒤에** 실행해야 한다 |
| External CAN Push 풀림 | 피드백이 1분쯤 오다가 0 | 웹에서 다시 누름 |
| 웹캠 창에서 `q` | 손 추적만 종료, 녹화는 저장되지 않고 끝남 | 키는 녹화 터미널을 클릭한 뒤 누른다 |
| `/dev/video` 번호 변경 | 카메라 재연결·SDK 시험마다 번호가 바뀜 (18 → 12 → 14, 손 추적 10 → 18 → 26) | 데이터 카메라는 녹화 전에 확인. 손 추적은 시리얼로 지정 |
| Orbbec 색상+깊이 리셋 | 레이저 5~6단계에서 카메라가 USB에서 빠짐 | 레이저 3단계 (§8). 전원 공급 허브면 재확인 |
| SDK가 다른 카메라의 드라이버를 떼어 감 | 시험 후 두 Orbbec의 MJPG 노드가 사라짐 | 시리얼 지정 시 V4L2 백엔드 (§8). 이미 떼어 갔으면 카메라 재연결 |
| pick-place 팔꿈치 꺾임 | §10의 15:11 | J4 ≥ 15°, J7 한계로 J2+J4 제한. 무작위 목표 6000스텝 시뮬레이션에서 꺾임 없음 |
| **Orca 손가락이 반대로 움직임** | 사람·시뮬레이터는 주먹인데 실물 손가락은 펴지거나 젖혀짐. 센서값은 명령과 1~5° 이내로 일치 | `config.yaml`의 `joint_to_motor_map`이 orca_core v2(Dynamixel) 복사본인데 이 손은 Feetech라 모터 방향이 전부 반대. 캘리브레이션도 "굽힘" 끝을 실제 폄 끝으로 기록해 스스로 일관됐기 때문에 센서값으로는 안 보였다. 17개 부호를 모두 뒤집고 손목 포함 재캘리브레이션 (`calibrate(force_wrist=True)`). 폄/45°/85° 명령을 데이터 카메라로 찍어 실제로 굽는 것 확인. 10/12 WSL 설정부터 그랬고, 손을 real로 움직인 건 이날이 처음 |
| **그 전 데이터의 손 값** | 16:38 이전 녹화의 손 관절값은 실물 손 모양과 반대 | `orca-real-test`, 그 전 `nero-orca-pick-real-test`, shadow 테스트들은 학습에 쓰지 않는다 |
| 긴 명령 붙여 넣기 | 줄 끝 `\`가 깨져 일부 인자가 별도 명령으로 실행됨 | `record_pick.sh` |
| 데이터셋 `frontal` 영상 | `observation.images.frontal`은 MuJoCo 렌더 | 실물 정책 학습에는 쓰지 않는 것이 좋다. 저장 끄는 옵션은 아직 없음 |

---

## 12. 카메라 배치

관찰(데이터) 카메라, `observation.images.data`:

- 모든 에피소드와 정책 실행 때 같은 위치에 고정하고 사진으로 기록한다.
- 작업 영역(트레이, 물체, 놓을 자리)과 로봇 손이 다 보이고, **조작자와 손 추적 카메라는 화면에 없게** 한다. 사람이 찍히면 정책이 사람 움직임을 단서로 배운다.
- 이날 최종 구도: 로봇 뒤쪽 위에서 비스듬히 내려다보는 시점, 트레이가 화면 가운데. 수평에 가까운 각도는 화면 절반 이상이 사무실 배경이었고, 오른쪽으로 돌리면 조작자가 들어왔다.
- USB 연장선을 쓰면서 USB 2.0(480 Mbps)으로 연결되어 CAN 어댑터·손 시리얼과 같은 허브 쪽에 붙었다. 640×480 MJPG 30 fps는 문제없지만 CAN 끊김과 함께 지켜볼 것.

손 추적 카메라 (Orbbec RGB-D):

- 손등 쪽 시점은 주먹을 쥘 때 손가락이 가려져 굽힘 추정이 나빠진다. 손바닥·손가락 안쪽이 보이도록 **손 높이보다 낮게 두고 30~45° 올려다보는** 배치가 좋다 (손바닥을 아래로 한 자연스러운 자세 그대로).
- 거리 40~70 cm (깊이 유효 범위 15 cm~1.5 m). 조작자를 마주 보면 `--mirror-lateral`.
- 비교 방법: 배치별로 shadow 20초 (`NAME=hand-view-a`, `hand-view-b`)를 녹화해 손가락 명령의 떨림과 주먹 굽힘 크기를 비교한다 (아직 안 함).

---

## 13. 남은 일

1. 손 추적 카메라 배치 비교 (§12).
2. 실제 수집: 작업 문구, 에피소드 수, 데이터셋 이름 결정 (`record_pick.sh`, `KEEP=1`).
3. CAN 어댑터 연결 안정화 (케이블, 포트, 전원 공급 허브).
4. 실물 수집 때 `frontal`(MuJoCo 렌더) 저장 끄기 옵션.
5. 트레이 바닥이 책상 면보다 낮으면 pick-place 아래 한계로는 손이 바닥에 닿지 못한다. 높이를 재서 반영.
6. Orca sync read 응답 누락 시 이전 값 유지 (WSL 문서 §8에서 이어짐).
7. `orca_teleop` 테스트 중 기존 실패 8개, 에러 2개 (`test_orca_hand_sink_go_home`, `test_recording_ready.py` 다수). 이번 수정 전에도 같았다.
8. `combined_sink.py --check`의 `index_mcp did not track the named command` (sim 손 추종 검사). 이번 수정 전에도 실패했다.
