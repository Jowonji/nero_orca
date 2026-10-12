# WSL2에서 Nero+Orca LeRobot 데이터 수집 환경 구성 기록

| 항목 | 내용 |
| --- | --- |
| 날짜 | 2026-10-12 |
| 범위 | Dell 노트북(Windows + WSL2)에서 실물 Nero 팔 + Orca 손 + Orbbec 카메라 2대를 연결하고 `record_dataset.py`로 LeRobot 데이터셋을 녹화 (shadow 모드까지) |
| 환경 | WSL2 (커널 6.18 microsoft), usbipd-win, USB-C 허브 2단 연결, Python 3.12 (`orca_teleop/.venv`), lerobot 0.4.4 |
| 선행 | [real-robot-bringup.md](real-robot-bringup.md), [jetson-webcam-teleop-first-run.md](jetson-webcam-teleop-first-run.md) |
| 결과 | shadow 모드 10초 에피소드 150프레임 녹화, `LeRobotDataset`으로 로드 확인. real 모드는 WSL의 CAN 불안정 때문에 보류 |

---

## 1. 요약

- 장치 4개(Nero CAN 어댑터, Orca 손 시리얼, Orbbec 2대)를 usbipd로 WSL에 붙여 shadow 모드 녹화까지 성공했다.
- 녹화 파이프라인 버그 3개를 고쳤다 (§4): 데이터셋 `finalize()` 누락, 하드웨어 읽기가 sink 잠금을 점유, Orca 손 51회 개별 읽기.
- Orca 손은 Dynamixel이 아니라 **Feetech** 모터다. 프로젝트 안에 손 설정·캘리브레이션을 따로 두었다 (`config/orcahand_right/`).
- WSL에서는 Orbbec 깊이 스트림이 안 되고(`--depth webcam` 사용), CAN 수신이 7~10분마다 멈춘다. **real 모드와 본격 수집은 네이티브 Linux에서 할 것.**

---

## 2. 장치 연결 (WSL2 + usbipd)

| 장치 | VID:PID | WSL 장치 | 비고 |
| --- | --- | --- | --- |
| Nero CAN 어댑터 (candleLight) | `1d50:606f` | `can0` (gs_usb) | 1 Mbit/s |
| Orca 손 (CH343) | `1a86:55d3` | `/dev/ttyACM0`, `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79029607-if00` | Feetech, 1 Mbit/s, ID 1~17 |
| Orbbec Gemini 335 (데이터 수집용) | `2bc5:0800` | 시리얼 `CP0H953001AR` | 녹화 `--camera data:N` |
| Orbbec Gemini 335 (손 추적용) | `2bc5:0800` | 시리얼 `CP0H95300129` | `--teleop-camera N` |

Windows 관리자 PowerShell:

```powershell
usbipd list                                   # BUSID는 꽂은 포트마다 바뀐다
usbipd bind --busid <BUSID>                   # 처음 한 번
usbipd attach --wsl --busid <BUSID>           # 카메라는 --auto-attach 권장
```

겪은 문제와 해결:

- **허브 USB 2.0 쪽 오류**: VIA 허브(`2109:2817`)가 Error 상태면 USB 2.0 장치(CH343, CAN 어댑터)가 Windows에서도 안 보인다. 허브를 다시 꽂아 해결.
- **2단 허브 포트 불량**: 두 번째 허브의 특정 포트에서 CAN 어댑터가 인식되지 않았다. 다른 포트로 옮겨 해결.
- **장치 재연결 시 attach 해제**: 허브를 건드리거나 카메라가 리셋되면 attach가 풀리고 `/dev/video*` 번호가 바뀐다. 녹화 전 아래로 번호를 확인한다.

```bash
for i in $(ls /dev/video* | sed 's#/dev/video##' | sort -n); do
  v4l2-ctl -d /dev/video$i --list-formats 2>/dev/null | grep -q MJPG && \
  echo "video$i $(udevadm info -q property -n /dev/video$i | grep ID_SERIAL_SHORT | cut -d= -f2)"
done
```

WSL에서 한 번 해 둘 것:

```bash
sudo apt install -y libsm6 libice6       # OpenCV Qt 창 (없으면 MediaPipe 프로세스가 창 열 때 abort)
sudo cp ~/workspace/orca_teleop/.venv/lib/python3.12/site-packages/pyorbbecsdk/shared/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER            # /dev/ttyACM0 권한 (WSL 재시작 후 적용)
```

---

## 3. Nero CAN

```bash
sudo ip link set can0 up type can bitrate 1000000
candump can0      # 0x2A1~0x2A9, 0x251~0x257 등이 초당 ~2100개 들어와야 정상
```

- **웹 컨트롤러(192.168.0.150)에서 `External CAN Push`를 눌러야 피드백이 들어온다.** 어댑터를 붙인 직후 0개였다가 버튼을 누른 순간부터 들어왔다. `Control Mode: WEB`이면 CAN으로 아무것도 안 온다.
- 웹 Linkage 탭에는 follower/leader만 있고 단독(normal) 모드가 없다. 현재 `Linkage Status: follower`인 채로도 CAN 읽기·쓰기 모두 동작했다.
- 펌웨어 질의(`get_firmware()`)에 `1.12`로 응답 → 송신·수신 모두 확인.
- **WSL 한계**: 7~10분 동작 후 수신이 멈추는 일이 4번 있었다. 이때 송신 echo도 돌아오지 않아(`tx_packets` 고정, 버스 에러 0) Nero가 아니라 usbipd/gs_usb 경로가 멈춘 것으로 판단했다. 복구: 어댑터를 물리적으로 뽑았다 꽂기 → attach → `can0` up → `External CAN Push`.

real 모드 단계 테스트 1단계(속도 10%, enable 후 현재 자세 유지 명령)는 성공했다: 관절 7개 enable, 제어 모드 `ETHERNET_CONTROL_MODE` → `CAN_CTRL`, 유지 중 편차 0.00°. 2단계(J7 ±2°)는 직후 CAN 수신이 멈춰 진행하지 않았다.

---

## 4. 코드 수정

### 4.1 `orca_teleop/scripts/record_dataset.py`: `dataset.finalize()` 추가

LeRobot 0.4는 에피소드 메타데이터를 버퍼에 모았다가 `finalize()`에서 쓴다. 호출이 없어서 `meta/episodes/`가 생기지 않았고, 데이터셋을 로드하면 로컬 파일을 못 찾고 Hub로 가서 실패했다. 종료 시 pyarrow `ImportDecimalType` 에러도 이것 때문이었다. `recorder_thread.join()` 직후에 `dataset.finalize()`를 호출한다.

### 4.2 `combined_sink.py`: 하드웨어 읽기를 잠금 밖으로

`dispatch_action()`이 `self._lock`을 쥔 채 Nero/Orca 피드백을 읽었다(Orca 시리얼 ~40 ms). 텔레옵 루프가 쉬지 않고 돌아 잠금을 거의 독점했고, 녹화 루프의 `get_observation()`이 밀려 10초에 38프레임만 기록됐다. 읽기를 잠금 밖으로 빼고 MuJoCo 갱신(`_advance`)만 잠금 안에 둔다.

### 4.3 `hardware.py`: Orca 손 Feetech sync read

`orca_core`의 `read_pos_vel_cur()`는 모터 17개 × (위치·속도·전류) = 51회 개별 읽기다. 같은 클라이언트에 있는 `read_pos_vel_cur_sync()`(GroupSyncRead 1회, 같은 스케일, 실패 시 개별 읽기로 폴백)로 바꿨다.

| 측정 (읽기 전용 / shadow) | 수정 전 | 수정 후 |
| --- | --- | --- |
| 손 읽기 1회 | 39.6 ms | 5.5 ms |
| `dispatch_action` 1회 | 46~47 ms | 7.1 ms |
| 실제 손 추적 녹화 (10 s, 15 Hz) | 38 / 70 프레임 | 150 프레임 |

부하가 클 때 가끔 한 모터가 sync read 응답에서 빠진다(`Motor 12 not available`). 이때 그 관절 값이 0이 된다. 개별 읽기도 실패 시 0이었고, 무부하 300회에서는 빠짐 없음. 빠진 경우 이전 값을 유지하도록 고치는 것이 남은 일이다.

---

## 5. Orca 손 설정 (`config/orcahand_right/`)

- `config.yaml`: `orca_core` v2 `orcahand_right` 복사본. `port`를 by-id 경로로, `motor_type: feetech` 추가. site-packages 원본은 건드리지 않는다 (재설치 시 덮어써지고, 연결 시 `orca_core`가 포트를 다시 써넣기도 함).
- `calibration.yaml`: 2026-10-12 새로 캘리브레이션 (`calibrated: true`, `wrist_calibrated: true`). Feetech는 첫 한계에서 모터 오프셋을 3595/500으로 다시 잡으므로 limit 값이 여러 모터에서 같은 것이 정상이다.
- 녹화·텔레옵 때 `--model-path .../config/orcahand_right/config.yaml`을 꼭 넘긴다. 기본값은 맥 포트 + Dynamixel이라 연결되지 않는다.
- 주의: `orca_core` 연결은 모든 모터 토크를 켜고 모드를 쓴다. shadow 모드에서도 손 토크가 켜진다 (`real-robot-bringup.md`의 "shadow는 토크 안 켬"과 다름).

캘리브레이션 명령:

```bash
cd ~/workspace/nero_orca
../orca_teleop/.venv/bin/python -c "
from orca_core import OrcaHand
h = OrcaHand(config_path='config/orcahand_right/config.yaml')
print(h.connect()); h.calibrate()"
```

---

## 6. Orbbec 깊이 (WSL에서 안 됨)

- 컬러(MJPG)만 스트리밍하면 정상이다.
- Orbbec SDK(libusb·V4L2 백엔드 모두)나 V4L2로 깊이(Z16) 스트림을 열면 프레임 0개 후 카메라가 리셋되고 usbipd attach가 풀린다.
- `--depth off`도 Orbbec SDK 경로를 쓰므로 같은 증상. **WSL에서는 `--depth webcam`** (일반 OpenCV 캡처, 깊이 없음).
- MediaPipe 카메라 자동 선택은 `/dev/video0~7`만 본다. 번호가 밀리면 못 찾으므로 `--teleop-camera`를 지정한다.

---

## 7. 녹화 절차와 명령

체크리스트: ① `usbipd list`에서 4개 모두 Attached ② `can0` up ③ 웹에서 `External CAN Push` → `candump can0` 확인 ④ 카메라 번호 확인 ⑤ 녹화. 녹화 중 웹 페이지 조작 금지.

```bash
cd ~/workspace/orca_teleop
.venv/bin/python scripts/record_dataset.py --backend combined --local --source mediapipe --show-video \
  --depth webcam --teleop-camera 14 --camera data:6 \
  --nero-output shadow --orca-output shadow \
  --model-path ~/workspace/nero_orca/config/orcahand_right/config.yaml \
  --urdf-path ~/workspace/orcahand_description/v1/models/urdf/orcahand_right.urdf \
  --repo-id local/nero-orca-shadow-test --task "shadow test" \
  --root ~/workspace/nero_orca/datasets/nero-orca-shadow-test \
  --fps 15 --num-episodes 1 --episode-seconds 10 --overwrite 2>&1 | tee ~/workspace/nero_orca/record_test.log
```

- 시작 후 손 추적 카메라에 오른손이 보여야 녹화가 시작된다 (`teleop=0/5`에서 멈춰 있으면 손 추적 문제).
- `--episode-end timer`(기본)는 Ctrl+C로만 중단, SPACE는 `--episode-end space|both`일 때만.
- 저장 항목: `observation.state`/`action` 각 24차원 (Nero 7 + Orca 17, 도 단위), `observation.images.data` (640×480), `observation.images.frontal` (MuJoCo 렌더 320×240).

---

## 8. 남은 일

1. real 모드 단계 테스트 2단계 이후 (네이티브 Linux에서).
2. Orca sync read 응답 누락 시 이전 값 유지.
3. 수집용 옵션 정리: `--episode-end space`, 에피소드 수, 태스크 문구.
4. 네이티브 Linux(수리 중인 PC 또는 Jetson)로 이전: `gs_usb`/`cdc_acm` 모듈, `mediapipe`·`pyorbbecsdk2` aarch64 설치 여부 확인, 깊이(`--depth auto`) 재확인.
