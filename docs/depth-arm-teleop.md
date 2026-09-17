# MediaPipe 팔 텔레오퍼레이션: depth 입력과 IK 구조

- 날짜: 2026-09-17
- 범위: 웹캠/Orbbec 입력으로 Nero 팔을 움직이는 경로(손목 위치 → 상대 이동 → IK). 손가락 retargeter 내부는 다루지 않음
- 선행: [nero-orca-attach-report.md](nero-orca-attach-report.md)
- 카메라: Orbbec Gemini 335 (WSL2 usbipd, `pyorbbecsdk2`는 `../orca_teleop/.venv`에 설치)
- 커밋: `orca_teleop` `webcam-teleop` 브랜치, `nero_orca` `main`

## 1. 전체 흐름

손가락과 팔은 **서로 다른 경로**로 움직인다. 기존 orca 파이프라인은 손가락만 담당하고, depth와 IK는 팔을 움직이기 위해 덧붙인 부분이다.

```
 카메라 (Orbbec 컬러 + depth)
        │
        ▼
 MediaPipe: 손 랜드마크 21개 검출  ← 기존과 동일, depth는 쓰지 않음
        │
        ├──[A] 손가락 경로 (기존 orca 파이프라인)
        │     world 랜드마크 → retargeter → Orca 손가락 관절 17개
        │
        └──[B] 팔 경로 (새로 추가)
              손목 위치 → 기준점 대비 이동량 → IK → Nero 팔 관절 7개
```

코드 경로:

`orca_teleop/.../mediapipe/publisher.py` (손목 위치 생성) → gRPC → `orca_teleop/.../ingress/server.py` (`HandLandmarks.wrist_position`, 3개 또는 6개 값) → `combined_sink.py` (`update_wrist_position`) → `arm_ik.py` (`WristPositionMirror`, `NeroArmIK`)

## 2. 기존 파이프라인만으로는 팔을 움직일 수 없는 이유

MediaPipe는 랜드마크를 두 종류로 준다.

- **world 랜드마크**: 손목을 원점(0,0,0)으로 한 미터 단위 좌표다. 손가락이 얼마나 굽었는지는 알 수 있지만 **손 전체가 공간에서 어디 있는지는 모른다.** 손을 1m 옮겨도 손목은 계속 원점이다. retargeter는 이것만 써서 손가락 각도를 계산한다.
- **이미지 랜드마크**: 화면 안의 픽셀 위치(0~1)다. 좌우·위아래는 알 수 있지만 **거리(앞뒤)는 모른다.**

그래서 팔용 **손목 위치(wrist position)** 를 따로 만들어 world 랜드마크 뒤에 붙여 보낸다. 코드에서는 `wrist_position`(gRPC 수신 값)과 `WristPosition`(`arm_ik.py`)이다.

일반적인 용어로는 다음에 해당한다.

| 이 문서 | 일반 용어 |
|---|---|
| 이미지 모드의 `x, y, palm_w` | 2D wrist keypoint + scale cue (scale-based monocular depth) |
| RGB-D 모드의 `X, Y, Z` | 3D wrist position (camera frame), deprojection으로 계산 |
| 기준점 대비 이동을 로봇에 더함 | relative (incremental) position mapping |
| 로봇 손 끝 목표 위치 | end-effector (EE) target position |

## 3. 앞뒤 거리를 얻는 두 가지 방법

### 3.1 이미지 모드 (`--depth off`): 값 3개

- 손목의 화면 위치 `x, y`와 손바닥 폭 `palm_w`(검지 뿌리와 새끼손가락 뿌리 사이 화면상 거리)를 보낸다 (`publisher.py`의 `_on_result`).
- 가까우면 손바닥이 크게 보이는 원리로 거리를 **추정**한다. 거리 변화는 `기준 palm_w / 현재 palm_w − 1`로 계산한다.
- 한계: 손바닥을 기울이거나 돌려도 폭이 작아 보인다. 이때 손이 멀어졌다고 착각한다. 단위도 미터가 아니라서 게인(`DEPTH_GAIN`, `LATERAL_GAIN`, `VERTICAL_GAIN`)을 손으로 맞춰야 한다.

### 3.2 RGB-D 모드 (`--depth orbbec`): 값 6개

MediaPipe가 찾은 픽셀 위치에서 depth 이미지 값을 **읽어오는** 방식이다 (`orbbec.py`의 `palm_point`).

1. Orbbec SDK의 `AlignFilter`가 depth 이미지를 컬러 이미지에 맞춰 정렬한다. 컬러의 (u, v) 픽셀이 depth의 같은 픽셀과 대응한다.
2. 손바닥 랜드마크 5개(손목, 검지·중지·약지·새끼 뿌리) 주변 7×7 픽셀에서 유효한 depth(0.15~1.5 m)를 모아 각각 중앙값을 낸다. 중앙값에서 6 cm 넘게 벗어난 값은 버리고 평균해 거리 `Z`를 구한다. 손가락 끝은 depth가 자주 비어서 쓰지 않는다.
3. 카메라 핀홀 모델로 손목 픽셀을 3D 좌표로 바꾼다.

   ```
   X = (u − cx) · Z / fx      (fx, fy, cx, cy: 컬러 카메라 내부 파라미터)
   Y = (v − cy) · Z / fy
   ```

4. `x, y, palm_w, X, Y, Z` 6개 값을 보낸다. 좌표는 컬러 카메라 기준(x 오른쪽, y 아래, z 앞)이고 단위는 m다. depth를 못 읽으면 X, Y, Z가 NaN이고, 이 샘플은 `arm_ik.py`에서 버린다(이미지 값으로 대체하지 않음).

즉 **MediaPipe가 "손이 어느 픽셀에 있는지"를 찾고, depth 카메라가 "그 픽셀까지 몇 m인지"를 알려준다.** depth를 MediaPipe 안에 넣는 것이 아니라 MediaPipe 결과에 덧붙이는 구조다.

## 4. 손 위치 → 로봇 목표 위치 (`arm_ik.py`의 `_offset`, `solve`)

카메라와 로봇 사이 캘리브레이션이 없으므로 절대 위치가 아니라 **상대 이동**만 쓴다.

1. 손이 처음 인식된 위치를 **기준점**으로 잡는다.
2. 이후에는 "기준점에서 얼마나 움직였나"를 로봇 좌표로 바꿔, 로봇 손 끝(EE, end-effector)의 기본 자세 위치에 더한다.

   | 카메라 | 로봇 |
   |---|---|
   | Z 증가 (멀어짐) | +X (앞) |
   | X 증가 (오른쪽) | +Y |
   | Y 증가 (아래) | −Z (아래) |

   RGB-D 모드는 `METRIC_SCALE`(기본 1.0, 손 1 m당 EE 1 m)을 곱한다.
3. 안정화:
   - **One Euro 필터**: 가만히 있을 때는 강하게 부드럽게 하고, 빠르게 움직일 때는 지연을 줄인다. 모드별로 파라미터가 따로 있다(`FILTER_*`, `METRIC_FILTER_*`).
   - **이동 한계** `POS_CLAMP`: 기본 자세 기준 X ±18 cm, Y ±25 cm, Z ±20 cm.
   - **최대 속도** `MAX_EE_SPEED` 0.8 m/s: 인식이 튀어도 팔이 갑자기 휘둘리지 않게 한다.
   - **손을 놓치면**(0.5 s 동안 새 손목 위치 없음, `WRIST_TIMEOUT`) 현재 자세를 유지하고, 다시 인식되면 그 자리를 새 기준점으로 잡아 튀지 않게 이어간다.

이 단계의 결과는 **"로봇 손 끝이 가야 할 xyz 좌표"** 하나다.

## 5. IK (Inverse Kinematics, 역기구학)

로봇 팔은 관절 7개이고, 실제로 명령할 수 있는 것은 **관절 각도**뿐이다.

- **FK (정기구학)**: 관절 각도 7개 → 손 끝 위치. 계산이 쉽다 (MuJoCo `mj_forward`).
- **IK (역기구학)**: 손 끝 위치 → 관절 각도 7개. 필요한 것은 이 방향인데, 식 하나로 풀리지 않아 반복 근사한다.

`_ik()`가 한 프레임에 최대 10번 반복하는 과정:

```
1. 현재 관절 각도로 손 끝 위치 계산 (FK)
2. 오차 err = 목표 위치 − 현재 위치
3. 자코비안 J: "각 관절을 조금 돌리면 손 끝이 어느 방향으로 얼마나 움직이나" (3×7 행렬)
4. 관절 변화량 dq = Jᵀ (J Jᵀ + λI)⁻¹ · err        ← damped least squares
   λ(DAMPING)는 팔이 쭉 펴진 특이 자세에서 dq가 폭주하지 않게 막는 작은 값
5. dq를 STEP_CLIP으로 자르고, 관절 한계 안으로 잘라 관절 각도에 더함
```

**Nullspace 항**: 위치는 3개(xyz)인데 관절은 7개라서 같은 손 끝 위치를 만드는 자세가 무수히 많다(팔꿈치를 어디 두든 손 끝은 같은 곳일 수 있다). 그래서 손 끝 위치에 영향을 주지 않는 방향 `(I − J⁺J)`으로만 기본 자세 쪽으로 살짝 당겨(`NULLSPACE_GAIN`) 팔꿈치가 떠돌지 않게 한다.

현재는 **위치만** 따라가고, 손목 방향(회전)은 기본 자세로 고정한다. 방향까지 따라가려면 카메라–로봇 캘리브레이션이 필요하다.

시뮬 쪽에서는 IK 결과가 15 Hz 계단처럼 바뀌지 않도록 물리 substep 사이에서 이전 명령과 새 명령을 선형 보간한다 (`combined_sink.py`).

## 6. 두 모드 비교

| 단계 | 손가락 | 팔 (이미지 모드) | 팔 (RGB-D) |
|---|---|---|---|
| 입력 | MediaPipe world 랜드마크 | 화면 x, y + 손바닥 폭 | 화면 위치 + depth 값 |
| 거리 정보 | 필요 없음 | 손바닥 크기로 추정 | 실측 (m) |
| 목표 | 손가락 관절 각도 | 손 끝 xyz | 손 끝 xyz |
| 변환 방법 | retargeter | 상대 이동 → IK | 상대 이동 → IK |

두 팔 모드는 **3장(거리를 어떻게 얻는지)만 다르고**, 그 뒤 필터·한계·IK는 같다.

같은 손 동작(카메라에서 60 cm)을 두 모드의 변환식에 넣었을 때 EE 목표 이동량(필터 전, `POS_CLAMP` 적용):

| 손 동작 | 이미지 모드 | RGB-D |
|---|---|---|
| 오른쪽 10 cm | 6.4 cm | 10 cm |
| 위로 10 cm | 6.4 cm | 10 cm |
| 멀어짐 20 cm | 8.3 cm | 18 cm (한계) |
| 가까워짐 20 cm | −8.3 cm | −18 cm (한계) |
| 제자리에서 손바닥만 기울임 (폭 −30%) | **앞으로 10.7 cm (오동작)** | **0 cm** |

좌우·위아래는 이미지 모드 게인(0.40 m / 화면 높이)이 60 cm 거리의 실제 크기와 비슷해서 체감 차이가 작다. 차이가 크게 나는 것은 **앞뒤 이동**과 **손바닥 기울임**이다.

## 7. 실행 방법 (전후 비교)

`--depth off`와 `--depth orbbec` 둘 다 Orbbec SDK로 같은 컬러 스트림(640×480@30)을 받는다. 손목 위치에 XYZ를 넣는지만 달라서 카메라 조건이 같다.

```bash
cd ~/workspace/nero_orca
export ORCAHAND_DESCRIPTION_DIR=$HOME/workspace/orcahand_description

# 이미지 모드 (RGB-D 전)
../orca_teleop/.venv/bin/python record_dataset.py --local --source mediapipe --depth off \
  --show-video --overwrite --fps 15 --episode-end space --num-episodes 5 \
  --urdf-path "$ORCAHAND_DESCRIPTION_DIR/v1/models/urdf/orcahand_right.urdf" \
  --task "move arm and flex fingers"

# RGB-D 모드 (RGB-D 후)
../orca_teleop/.venv/bin/python record_dataset.py --local --source mediapipe --depth orbbec \
  --show-video --overwrite --fps 15 --episode-end space --num-episodes 5 \
  --urdf-path "$ORCAHAND_DESCRIPTION_DIR/v1/models/urdf/orcahand_right.urdf" \
  --task "move arm and flex fingers"
```

| `--depth` | 카메라 | 손목 위치 값 | 기본 저장 위치 |
|---|---|---|---|
| `auto` (기본) | Orbbec 있으면 SDK, 없으면 웹캠 | 6 또는 3 | `datasets/nero-orca-sim-mediapipe` |
| `orbbec` | Orbbec SDK (없으면 에러) | 6 | `datasets/nero-orca-sim-mediapipe-depth-orbbec` |
| `off` | Orbbec SDK 컬러 | 3 | `datasets/nero-orca-sim-mediapipe-depth-off` |
| `webcam` | OpenCV 웹캠 | 3 | `datasets/nero-orca-sim-mediapipe-depth-webcam` |

차이가 잘 드러나는 테스트 동작:

1. 제자리에서 손바닥만 기울이거나 돌리기: `off`는 팔이 앞으로 나가고, `orbbec`은 가만히 있어야 한다.
2. 카메라 쪽으로 20 cm 다가갔다가 돌아오기: `orbbec`이 앞뒤로 두 배 넘게 움직인다.
3. 손가락 쥐었다 펴기: `off`에서는 손바닥 폭이 흔들려 팔이 앞뒤로 조금 흔들릴 수 있다.

주의:

- 에피소드는 **스페이스바**로 끝내야 저장된다. Ctrl+C로 바로 끄면 `total_episodes: 0`으로 남는다.
- 손이 처음 인식된 위치가 기준점이므로 매번 같은 자리에서 시작한다.
- `--overwrite`는 같은 모드의 기존 데이터셋을 지운다.
- WSL2 usbipd 환경에서 Orbbec 로그에 `Update data size > data buffer size` 경고가 자주 나온다. 컬러 프레임이 일부 버려져 실제 속도는 20~25 fps 정도다.

## 8. 남은 일

- 프레임마다 손목 위치 원본 값, 필터 후 값, EE 목표, 실제 EE 위치를 CSV로 저장해 두 모드를 정량 비교
- `METRIC_SCALE`, `METRIC_FILTER_*`, `POS_CLAMP` 실측 튜닝
- 카메라–로봇 캘리브레이션 후 손목 방향(회전) 추종
