# 네이티브 Linux(듀얼부팅)에서 Nero+Orca LeRobot 데이터 수집 환경 구성 기록

| 항목 | 내용 |
| --- | --- |
| 날짜 | 2026-10-14 |
| 범위 | Dell 노트북 Ubuntu 듀얼부팅에서 실물 Nero 팔 + Orca 손 + Orbbec 2대로 `record_dataset.py` 녹화 (shadow 검증 → real 단계 시작) |
| 환경 | Ubuntu 24.04, 커널 7.0, X11, Intel Iris Xe, Python 3.12 (`orca_teleop/.venv`, uv), lerobot 0.4.4, torch 2.10 |
| 선행 | [wsl-lerobot-recording-setup.md](wsl-lerobot-recording-setup.md), [real-robot-bringup.md](real-robot-bringup.md), [jetson-webcam-teleop-first-run.md](jetson-webcam-teleop-first-run.md) |
| 결과 | shadow 10초 에피소드 150/150 프레임, 실물 센서값 저장 확인. real 모드(팔만, 속도 10%) 실행 시작 |

---

## 1. 요약

- WSL에서 막혔던 CAN 끊김과 usbipd 문제는 네이티브 Linux에서 사라졌다. 장치는 꽂으면 바로 잡힌다.
- 설치 중 패키지 충돌 2건(`av`, OpenCV)을 정리했다 (§2.3).
- 녹화 데이터 문제 3개를 고쳤다 (§4).
  - Orca 손 캘리브레이션이 어긋나 값이 한계로 잘림 → 재캘리브레이션
  - `observation.state`가 MuJoCo 모델값이라 손 MCP가 -24.4°에서 잘림 → 하드웨어 모드는 센서값 저장
  - 녹화가 15 Hz를 못 채움 (10초에 75~121프레임) → retargeter 속도 제한
- 화면 배치: 웹캠 창은 왼쪽 위, MuJoCo 창은 오른쪽에 사이드 패널 없이, 로봇에 맞춘 시점으로 뜬다 (§5).

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

- `--episode-end space`: 스페이스바로 저장하고 다음 에피소드, `q`/Esc로 종료.
- 2026-10-14 14:09에 1단계(팔만 real, 10%)를 실행했다. 결과는 아직 기록하지 않음.

---

## 8. 남은 일

1. real 1~2단계 결과 기록.
2. Orca sync read 응답 누락 시 이전 값 유지 (WSL 문서 §8에서 이어짐).
3. 네이티브 Linux에서 `--depth auto` 확인.
4. `orca_teleop` 테스트 중 기존 실패 8개, 에러 2개 (`test_orca_hand_sink_go_home`, `test_recording_ready.py` 다수). 이번 수정 전에도 같았다.
5. `combined_sink.py --check`의 `index_mcp did not track the named command` (sim 손 추종 검사). 이번 수정 전에도 실패했다.
