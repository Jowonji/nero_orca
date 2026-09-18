# 실물 Nero+Orca 연결 준비 상태와 첫 연결 절차

- 날짜: 2026-09-17
- 범위: Python(pyAgxArm, orca_core)으로 실물 Nero 팔과 Orca 손을 연결하고 웹캠 텔레오퍼레이션으로 움직이기까지. 학습·작업 과제는 다루지 않음
- 선행: [depth-arm-teleop.md](depth-arm-teleop.md)
- 관련 코드: `hardware.py`, `combined_sink.py`, `mirror_real.py`, `../orca_teleop/scripts/record_dataset.py`
- 상태: 코드는 가짜 장치로만 검증했고 **실물 하드웨어에서는 아직 실행하지 않음**

## 1. 결론

연결해서 **읽기 → shadow → 저속으로 팔 움직이기**까지는 시도할 수 있다. 물체를 집는 등 실제 작업은 아직 안 된다. 손목 방향 고정, 좁은 이동 범위, 카메라 축 보정 부재 때문이다.

| 단계 | 가능 여부 | 설명 |
|---|---|---|
| 실물 관절 읽기 + 뷰어 (`mirror_real.py`) | 바로 시도 가능 | 읽기만 해서 위험 없음 |
| 웹캠 명령을 고스트로만 보기 (`shadow`) | 가능 | 명령을 보내지 않음 |
| 웹캠으로 실물 팔·손 움직이기 (`real`) | 코드 있음, 확인 후 저속으로 | 2장 확인 항목 통과 후 |
| 물체 집기 등 실제 작업 | 아직 안 됨 | 손목 방향 추종, 카메라 축 보정, 범위 확대 필요 |

출력 모드는 팔(`--nero-output`)과 손(`--orca-output`)에 각각 지정한다.

| 모드 | 하드웨어 연결 | 명령 전송 | 뷰어 |
|---|---|---|---|
| `sim` (기본) | 안 함 | 안 함 | 시뮬 상태 |
| `shadow` | 읽기만 (enable·토크 안 켬) | 안 함 | 실물 상태 + 반투명 명령 고스트 |
| `real` | enable, 토크 켬 | 함 | 실물 상태 + 반투명 명령 고스트 |

## 2. `real` 전에 확인할 것

1. **CAN 연결**: WSL에서 `can0`이 잡히는지 확인한다. 안 되면 Windows에서 실행한다(pyAgxArm Windows 드라이버 `agx_cando`).
2. **관절 방향·0점**: `mirror_real.py`에서 실물 팔을 움직였을 때 뷰어도 같은 방향으로 움직여야 한다. 반대면 `hardware.py`의 `NERO_JOINT_SIGN`, 일정한 차이가 있으면 `NERO_JOINT_OFFSET`을 고친다. 고치기 전에 `real`을 쓰면 엉뚱한 방향으로 움직인다.
3. **Orca 손 캘리브레이션**: v2 설정의 `calibration.yaml`이 `calibrated: false`다. 손을 `real`로 쓰면 시작할 때 모터를 끝까지 움직이는 캘리브레이션이 자동 실행된다. 처음에는 손을 `sim`으로 두고 팔만 확인한다.
4. **시작 동작**: `real`로 켜면 팔이 기본 자세(`REST_QPOS`)로 먼저 이동한다. 주변을 비우고 비상정지를 손에 둔다.
5. **연속 명령 반응**: 15 Hz로 보내는 `move_j`를 부드럽게 따라가는지 `--nero-speed 10`으로 작게 움직이며 확인한다.

`real` 모드의 코드 안전장치(`hardware.py`):

- 관절 한계 안으로 명령을 자름 (SDK 한계에서 1° 안쪽)
- 관절 속도 0.6 rad/s 제한
- 명령과 실제 자세 차이가 20°를 넘으면 전송 보류
- 관절 피드백이 0.25 s 넘게 갱신되지 않으면 전송 보류
- 종료 시 Nero는 enable 상태 유지 (손 달린 팔이 떨어지지 않게). 안전 자세에서 받친 뒤 disable

## 3. 첫 연결 절차 (예상 1~2시간)

```bash
cd ~/workspace/nero_orca
export ORCAHAND_DESCRIPTION_DIR=$HOME/workspace/orcahand_description

# 1) Windows PowerShell(관리자): CAN 모듈을 WSL로 넘기기
#    usbipd list
#    usbipd bind --busid <CAN 모듈 BUSID>
#    usbipd attach --wsl --busid <CAN 모듈 BUSID>

# 2) CAN 활성화 (sudo 비밀번호 필요)
sudo ip link set can0 up type can bitrate 1000000

# 3) 읽기만: 팔만 (관절 방향·0점 확인)
../orca_teleop/.venv/bin/python mirror_real.py --orca sim

# 4) shadow: 웹캠 명령을 고스트로 확인
../orca_teleop/.venv/bin/python record_dataset.py --local --source mediapipe --depth orbbec \
  --show-video --nero-output shadow --orca-output sim --num-episodes 1 --episode-end space \
  --urdf-path "$ORCAHAND_DESCRIPTION_DIR/v1/models/urdf/orcahand_right.urdf" --task "shadow test"

# 5) real: 팔만, 최저속
../orca_teleop/.venv/bin/python record_dataset.py --local --source mediapipe --depth orbbec \
  --show-video --nero-output real --nero-speed 10 --orca-output sim --num-episodes 1 --episode-end space \
  --urdf-path "$ORCAHAND_DESCRIPTION_DIR/v1/models/urdf/orcahand_right.urdf" --task "real arm test"
```

- 3번에서 관절 방향이 틀리면 거기서 멈추고 보정한다.
- 4번에서는 고스트가 손을 따라 자연스럽게 움직이는지 본다.
- 5번은 손을 아주 작게 움직인다.
- 하드웨어 없이 뷰어만 확인할 때는 `mirror_real.py --fake`를 쓴다.

## 4. 알아둘 한계

- **팔은 위치만 따라간다.** 손목 방향은 고정이고, 이동 범위는 기본 자세 기준 X ±18 cm, Y ±25 cm, Z ±20 cm다.
- **카메라와 로봇 축이 같다고 가정한다.** 첫 테스트 때는 Orbbec을 로봇과 같은 방향을 보게 둔다. 비스듬하면 손을 앞으로 내밀어도 팔이 옆으로 갈 수 있다.
- **하드웨어 모드로 녹화해도 `observation.images.frontal`은 MuJoCo 렌더다.** 학습용 데이터에는 `--camera`로 실제 카메라 영상을 추가해야 한다.
- **orca_teleop 환경의 orca_core는 0.2.1**이고 기본 설정이 v1이다. 실물 손 기본값은 결합 모델과 같은 v2 설정(`../orca_core/orca_core/models/v2/orcahand-right/config.yaml`)으로 바꿔 두었다. 실제 손이 v1이면 `--model-path`로 지정한다.

## 5. 무게와 가반하중

### 5.1 사양

| 항목 | 값 | 비고 |
|---|---|---|
| Nero 가반하중 | 3.0 kg | 기준 자세·무게중심 거리는 공개 자료에 없음 |
| Nero 자체 무게 / 도달 거리 / 반복정밀도 | 4.8 kg / 580 mm / 0.1 mm | |
| Orca 손 무게 (공개 자료) | 논문 약 1.2 kg, 판매처 1.1~1.3 kg | v1 기준, 전완 타워 포함 여부 불명확. 보유한 손은 v2 |
| Orca 손 무게 (현재 MuJoCo 모델) | 1.764 kg (전완 타워 포함, 부품 20개) | 질량이 명시되지 않아 메시 부피 × 물 밀도로 자동 계산한 값 |
| Orca 파지 유지력 | 네 손가락 10.5 kg (103 N), 검지만 2 kg | 물체를 쥐고 당기는 시험값. 들 수 있는 무게가 아님 |

- 현재 모델에서 손 무게중심은 `link7` 플랜지에서 약 9.6 cm 떨어져 있다.
- 손이 들 수 있는 물체 무게의 한계는 **팔 가반하중 − 손 무게 ≈ 1.2~1.8 kg**이다. 손 파지력은 제한 요인이 아니다.
- **실제 로봇은 손 무게를 모른다.** pyAgxArm Nero 드라이버에는 부하 설정 함수가 없다(Piper 드라이버에만 `set_payload`가 있음).

### 5.2 자세별 관절 부하 (모델 계산)

중력에 의한 관절 토크를 계산했다. 비율은 SDK의 MIT 모드 토크 제한값(1·2번 24, 3·4번 16, 5~7번 8 N·m) 대비다.

| 자세 | 팔 끝 무게 | 2번 관절 | 4번 관절 | 제한 대비 최대 |
|---|---|---|---|---|
| 기본 자세 (팔꿈치 굽힘) | 손 1.76 kg | 7.2 N·m | 7.2 N·m | 45% |
| 기본 자세 | 손 1.76 + 물체 1.24 kg (합 3 kg) | 11.4 N·m | 11.4 N·m | 71% |
| 팔 수평으로 뻗음 | 손 1.76 kg | 20.1 N·m | 8.3 N·m | 84% |
| 팔 수평으로 뻗음 | 손 1.76 + 물체 1.24 kg (합 3 kg) | 29.2 N·m | 13.6 N·m | 122% (초과) |

- 팔을 굽힌 자세에서는 3 kg도 여유가 있지만, 수평으로 뻗으면 손만 달아도 84%다. 작업대를 로봇 가까이 두고 팔을 뻗는 자세를 피한다.
- 제한값은 모터 정격 토크가 아니고 손 무게도 추정치다. 절대 수치보다 자세별 경향으로 본다.

### 5.3 무게 보정

실측 후 모델 질량을 보정한다. 실물 위치 제어(`move_j`)와 IK에는 영향이 없지만, 자세별 부하 계산, 시뮬의 팔 처짐, 이후 Real-to-Sim 정확도에 필요하다.

1. 손 실측 (전완 타워 포함, 케이블 제외). 가능하면 무게중심 위치도 잰다.
2. 부품별 비율은 유지하고 손 전체 질량을 실측값으로 맞춘다.
3. 5.2 표를 실측값과 실제 작업 자세로 다시 계산한다.
4. 제조사에 3 kg 가반하중 기준 조건(무게중심 거리, 자세)과 관절 정격 토크를 확인한다.
5. 실물 연결 후 `get_motor_states()`로 기본 자세와 작업 자세의 관절 토크를 읽어 계산과 비교한다.

## 출처

- [AgileX NERO 공식 제품 페이지](https://global.agilex.ai/products/nero)
- [ORCA Hand 논문 (arXiv:2504.04259)](https://arxiv.org/html/2504.04259)
- [ORCA Hand 사양 (RoboZaps)](https://robozaps.com/products/orca-hand)
- [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm), [CAN 설정 문서](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/can_user.md)
