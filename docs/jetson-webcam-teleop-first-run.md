# Jetson Thor 실물 Nero+Orca 웹캠 텔레옵 첫 연결 기록

| 항목 | 내용 |
| --- | --- |
| 날짜 | 2026-09-18 |
| 범위 | Jetson Thor에서 실물 Nero 팔 + Orca 손을 웹캠 텔레옵으로 첫 연결 (sim → shadow → real 순서) |
| 환경 | Jetson AGX Thor (tegra 커널), Ubuntu, Python 3.12, GPU 렌더(EGL) |
| 선행 | [real-robot-bringup.md](real-robot-bringup.md), [depth-arm-teleop.md](depth-arm-teleop.md) |
| 카메라 | Orbbec Gemini 335 |
| 결과 | 팔+손 real 모드 웹캠 텔레옵 동작 확인. `hardware.py` tracking-error 데드락 버그 발견 및 수정 |

---

## 1. 요약 (TL;DR)

- 기존 문서(`real-robot-bringup.md`, `orca_teleop/docs/webcam-teleop-report.md`)는 WSL2 기준이라, 네이티브 Linux인 Jetson Thor에서는 몇 가지가 다르다. §2에 정리.
- `mirror_real.py`로 관절 1~7번 방향을 손으로 직접 확인 → 전부 실물과 뷰어가 같은 방향으로 움직여 `NERO_JOINT_SIGN`/`NERO_JOINT_OFFSET`은 기본값(항등) 그대로 둔다.
- `teleop.py --nero-output real --orca-output real`로 실물 팔+손을 웹캠으로 조작하는 데 성공했다.
- 그 과정에서 `hardware.py`의 tracking-error 안전장치가 한 번 걸리면 다시는 안 풀리는 구조적 버그를 발견해 수정했다 (§4.4).

---

## 2. Jetson Thor vs WSL2 차이

| 항목 | WSL2 (기존 문서) | Jetson Thor (이번) |
| --- | --- | --- |
| CAN 활성화 | 매번 `sudo ip link set can0 up` | `nero-can.service`가 부팅 시 자동으로 1Mbps로 올려둠. 확인만 하면 됨 |
| USB 패스스루 | Windows에서 `usbipd attach` 필요 | 불필요 (네이티브 USB) |
| 웹캠 권한 | `newgrp video` 매 터미널 필요 | 계정이 이미 `video` 그룹, 불필요 |
| MuJoCo 렌더 | `llvmpipe` 소프트웨어 렌더 (~15fps) | NVIDIA GPU, `MUJOCO_GL=egl`로 25~26fps |
| GUI 표시 | WSLg가 자동 처리 | 모니터가 Jetson에 HDMI로 직결 (`DISPLAY=:1`); SSH 세션엔 자동 설정 안 됨 |

---

## 3. 디스플레이 설정 (SSH로 접속해 실행할 때)

SSH로 Jetson에 붙어 실행하면 `DISPLAY`가 비어 있어 MuJoCo 뷰어(GLFW)가 다음 에러로 안 뜬다.

```
GLFWError: (65550) b'X11: The DISPLAY environment variable is missing'
ERROR: could not initialize GLFW
```

확인 절차:

1. `who` / `loginctl list-sessions` → seat0에 GDM 그래픽 세션이 `:1`로 떠 있음을 확인.
2. `DISPLAY=:1 XAUTHORITY=/run/user/2001/gdm/Xauthority xrandr --query` → `HDMI-0 connected primary 1920x1080` 으로 실제 모니터가 물려 있음을 확인.

해결: SSH 세션에서 아래 두 변수를 지정하면 뷰어가 Jetson에 직결된 물리 모니터에 뜬다.

```bash
export DISPLAY=:1
export XAUTHORITY=/run/user/2001/gdm/Xauthority
```

(참고: `2001`은 이 계정의 UID. 다른 계정이면 `/run/user/<UID>/gdm/Xauthority`로 바뀐다.)

---

## 4. 실행 순서와 결과

### 4.1 환경 변수 (공통)

```bash
cd ~/workspace/nero_orca
export DISPLAY=:1
export XAUTHORITY=/run/user/2001/gdm/Xauthority
export ORCAHAND_DESCRIPTION_DIR=$HOME/workspace/orcahand_description
export MUJOCO_GL=egl
```

### 4.2 관절 방향 확인 (읽기 전용, 안전)

```bash
../orca_teleop/.venv/bin/python mirror_real.py --orca sim
```

- CAN 연결 정상: `Nero connected on socketcan:can0`
- 토크가 꺼진 채로 팔이 중력에 늘어진 상태였다 — 이건 정상이다(REST 자세 고스트와 다른 게 당연함). "고스트와 자세가 맞는지"가 아니라 **손으로 움직였을 때 방향이 맞는지**만 확인하면 된다.
- 관절 1~7번을 하나씩 손으로 움직여 방향 확인 → 전부 실물과 뷰어가 같은 방향으로 움직임 → sign/offset 수정 불필요.

### 4.3 웹캠 shadow 테스트 (팔 shadow, 손 sim — 로봇에 명령 안 나감)

```bash
../orca_teleop/.venv/bin/python teleop.py
```

- Orbbec RGB-D SDK가 USB 디바이스를 열지 못해(`usbEnumerator openUsbDevice failed`) 깊이 없는 OpenCV 웹캠 모드로 자동 전환됨 (§6 참고).
- 리타겟 25~26 fps. 손가락 추적, 팔 고스트(손목 위치 추종) 모두 정상 확인.

### 4.4 실물 팔 real 테스트 — tracking-error 데드락 버그

```bash
../orca_teleop/.venv/bin/python teleop.py --nero-output real --nero-speed 10
```

첫 시도에서 확인 프롬프트를 통과하고 팔이 REST로 이동을 시도했지만, 그 뒤로 팔이 전혀 움직이지 않았다. 로그:

```
hardware | Nero command held: Nero tracking error 98.6 deg
hardware | Nero command held: Nero tracking error 98.7 deg
... (계속 반복, 줄어들지 않음)
```

**원인 분석**

`combined_sink.connect()`는 `NeroOutput.move_home()`으로 팔을 `REST_QPOS`로 옮긴다. 이번엔 시작 자세가 토크 꺼짐으로 크게 늘어져 있었다(관절2 105° vs REST 0°, 관절4 −9.6° vs REST 69.9°, 관절7 −7° vs REST 75°). 10% 속도로는 `NERO_HOME_TIMEOUT`(20s) 안에 REST에 도달하지 못했는데, `move_home()`은 **도달 여부와 상관없이** 다음처럼 무조건 `_last_sent`를 기록한다.

```python
self._last_sent = target.copy()  # REST_QPOS, 실제로 도달했는지는 확인 안 함
```

이후 매 프레임 호출되는 `NeroOutput.send()`는 `base = self._last_sent`(이미 REST에 도달했다고 착각한 값)를 기준으로 다음 스텝을 계산한다. `target(base+step)`이 사실상 base와 같으므로, 실제 위치(`q`)와의 차이(tracking error)는 계속 ~100°로 유지된다. `err > NERO_MAX_TRACKING_ERROR`(20°)이면 `send()`는 아무 것도 보내지 않고 hold만 하는데, **hold 시 `_last_sent`를 갱신하지 않으므로 base가 영원히 고정된다.** 한 번 이 상태에 빠지면 실제 로봇 위치가 어떻든 다시는 명령이 나가지 않는 구조적 데드락이었다.

두 번째 시도(손 real 포함)에서도 같은 증상이 재현됐다. 이번엔 이전 세션에서 REST에 어느 정도 가까워진 상태(약 22° 차이)로 시작했는데도, 웹캠으로 손을 움직여 IK 목표가 계속 바뀌자 오차가 22°→25°로 벌어지며 다시 영구 hold에 빠졌다.

**수정** (`hardware.py`, `NeroOutput.send()`)

```python
err = float(np.max(np.abs(target - q)))
if err > NERO_MAX_TRACKING_ERROR:
    # base may be stale (e.g. an unreached move_home target); re-anchor on
    # the arm's actual position so the next call steps from where it
    # really is instead of holding forever against a gap that can
    # never close (base was frozen and never advances while held).
    self._last_sent = None
    return self._hold(f"Nero tracking error {np.degrees(err):.1f} deg")
```

`_last_sent`를 `None`으로 리셋하면, 다음 호출에서 `base = self._last_sent if self._last_sent is not None else q`에 의해 **실제 현재 위치(`q`)** 를 기준으로 다시 스텝을 계산한다. err는 한 스텝 크기(최대 `NERO_MAX_JOINT_SPEED * dt` ≈ 0.6 rad/s만큼)로 줄어들어 안전 범위 안에 들어오고, 다음 프레임부터 목표를 향해 다시 조금씩 이동한다.

수정 후 재시도: 초반에 hold 로그가 몇 프레임 찍히다가 곧바로 회복되어, 팔이 웹캠 손 위치를 계속 따라 움직이는 것을 확인했다.

### 4.5 실물 팔+손 real 테스트

```bash
../orca_teleop/.venv/bin/python teleop.py --nero-output real --orca-output real --nero-speed 10
```

- Orca 손이 `calibrated: false` 상태라 처음 켤 때 자동 캘리브레이션 스윕이 실행됨(모터가 각 관절을 끝까지 움직임).
- 캘리브레이션 이후 손가락이 웹캠 손 모양을 정상적으로 따라감.
- §4.4 수정을 적용한 상태로 팔과 손이 동시에 웹캠 동작을 따라 움직이는 것을 확인.

---

## 5. 안전 메모

- `real` 모드를 끈 뒤(`Ctrl+C`)에도 Nero는 **토크가 켜진 채로 유지**된다 (`hardware.py` 주석: "손 달린 팔이 떨어지지 않게"). 다음 실행 전에 팔을 만지기 전에는 뻣뻣한지(토크 on) 먼저 확인할 것.
- 팔이 토크 꺼짐으로 축 늘어진 상태에서 곧바로 `real`로 크게(80~100°) 이동시키는 것은 §4.4 버그가 아니어도 위험할 수 있다. 가능하면 토크 꺼진 상태에서 손으로 REST 자세 근처까지 옮겨둔 뒤 `real`을 켜는 걸 권장한다.
- `--nero-speed`는 낮을수록 안전하지만, 시작 자세가 REST에서 많이 벗어나 있으면 `NERO_HOME_TIMEOUT`(20s) 안에 못 갈 수 있다. §4.4 수정 이후에는 못 가도 데드락 없이 회복되므로 치명적이진 않다.

---

## 6. 남은 문제: Orbbec RGB-D SDK 미작동

```
orca_teleop.ingress.mediapipe.orbbec | Orbbec RGB-D unavailable (usbEnumerator openUsbDevice failed!); using the OpenCV webcam without depth
```

- `lsusb`로 Orbbec Gemini 335가 인식되는 것은 확인했다 (`2bc5:0800 Orbbec Gemini 335`).
- 커널이 UVC 웹캠으로 먼저 점유해서 `pyorbbecsdk2`가 raw USB로 열지 못하는 전형적인 Orbbec Linux 이슈로 추정된다.
- 현재는 `--depth auto`가 자동으로 깊이 없는 웹캠 모드로 폴백해 동작은 하지만, [depth-arm-teleop.md](depth-arm-teleop.md)에서 설명한 RGB-D 실측 거리(6값)를 못 쓰고 있어 팔 앞뒤 이동 정확도가 떨어진다.
- 다음에 해결할 것: Orbbec udev 규칙 설치(OrbbecSDK의 `install.sh` 스크립트), 필요시 `v4l-utils` 설치 후 어느 프로세스가 노드를 점유하는지 확인.

---

## 7. 빠른 참조

```bash
cd ~/workspace/nero_orca
export DISPLAY=:1
export XAUTHORITY=/run/user/2001/gdm/Xauthority
export ORCAHAND_DESCRIPTION_DIR=$HOME/workspace/orcahand_description
export MUJOCO_GL=egl

# 관절 방향 확인 (읽기 전용)
../orca_teleop/.venv/bin/python mirror_real.py --orca sim

# 웹캠 shadow (팔 미작동, 안전)
../orca_teleop/.venv/bin/python teleop.py

# 웹캠 real (팔+손 실제 구동, 저속)
../orca_teleop/.venv/bin/python teleop.py --nero-output real --orca-output real --nero-speed 10
```
