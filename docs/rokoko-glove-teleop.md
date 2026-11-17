# Rokoko Smartgloves → Orca hand (sim)

2026-11-16/17. Rokoko Smartgloves(오른손) 실시간 스트림을 리눅스 노트북에서 받아 Orca 핸드 관절
목표값으로 바꾸고 MuJoCo에서 확인한 뒤, `teleop.py --source rokoko` / `record_dataset.py --source rokoko`
/ `SOURCE=rokoko ./record_pick.sh`로 기존 경로(sink는 동일)에 연결한 기록. 11/17 실물 Orca 핸드로 확인:
시뮬과 실물이 같이 움직인다.

## 구성

```
Windows 노트북: Rokoko Studio ─ UDP JSON v3 (LZ4 없음), 30 Hz ─▶ 이 노트북 :14043
                                                                  rokoko/stream.py   RokokoReceiver (최신 패킷만)
                                                                  rokoko/retarget.py RokokoHandRetargeter
                                                                  rokoko/play.py     MuJoCo 뷰어 (녹화/실시간)
```

- 두 노트북은 같은 아이폰 핫스팟. 이 노트북 IP는 `hostname -I`, ufw는 `14043/udp` 허용.
- Studio Custom Streaming: UDP, JSON v3, LZ4 끔, 대상 = 이 노트북 IP:14043. 현재 Studio 설정이
  30 Hz로 보내며(60이 아님) 텔레옵에는 충분하다.
- 프레임 하나가 패킷 하나(약 10 KB), `scene.actors[0].body[관절]` = `position`(m) + `rotation`(xyzw).
  전신 63개 관절 중 오른손 21개(`rightHand`, `right{Thumb,Index,Middle,Ring,Little}{Proximal,Medial,Distal,Tip}`)만 쓴다.

## 명령

```bash
cd ~/workspace/nero_orca
PY=../orca_teleop/.venv/bin/python

$PY rokoko/listen.py --seconds 10                 # 수신 확인: 관절 이름, 값 범위, Hz
$PY rokoko/record.py --seconds 30 --out rokoko/samples/x.npz   # 샘플 녹화 (.npz)
$PY rokoko/play.py --file rokoko/samples/x.npz --loop          # 녹화 재생 → MuJoCo
$PY rokoko/play.py --live                                       # 실시간 → MuJoCo (단독 뷰어)

export ORCAHAND_DESCRIPTION_DIR=$HOME/workspace/orcahand_description
$PY teleop.py --source rokoko --nero-output sim --orca-output sim   # 기존 텔레옵 경로로 (sim)
```

`teleop.py --source rokoko`는 `rokoko/worker.py`의 `run_rokoko()`를 쓴다: MediaPipe 인그레스·리타게터
스레드 대신 글러브 스레드 하나가 `OrcaJointPositions`를 `actions_q`에 넣고, `CombinedNeroOrcaSink`는
그대로다. 손목 위치 입력이 없으므로 **팔은 홈 자세에 머물고 손만 움직인다.**

**Orca 손목 굽힘**은 `--pick-place`에서 웹캠과 같은 방식으로 따라온다: 글러브의 손목→중지 MCP 방향
기울기(`rokoko.retarget.hand_pitch`, 아래가 +)를 `sink.update_hand_pitch()`로 넘기면
`WristPositionMirror.update_hand_pitch()`가 고정 cue(`HOLD_CUE`) + 피치로 `WristPosition`을 만들고,
기존 pick 모드 로직(첫 샘플을 기준각으로 잡고 상대 기울기를 `PICK_WRIST_RANGE` −40~25°로)이 그대로 돈다.
cue가 고정이라 팔 오프셋은 0 — sink 경로 테스트에서 손목 0→+25→−15°로 따라가는 동안 팔 명령 변화 0°.
pick 모드가 아니면 손목은 0° 그대로. 팔 위치 입력은 VIVE Ultimate Tracker로 할 계획이지만, 그 전 테스트용으로
**`--rokoko-arm`**을 주면 글러브의 `rightHand` 위치(Studio의 전신 추정, m)를 웹캠 깊이 모드와 같은 경로로
팔 IK에 넣는다 (`rokoko.retarget.wrist_camera_xyz` → `sink.update_hand_pose` → `WristPositionMirror.update_metric`
→ 기존 metric 분기: 첫 샘플 기준 상대 이동 × `METRIC_SCALE` 1.5, pick 클램프·속도 제한 그대로).
Rokoko 월드(오른쪽 +x, 위 +y, 앞 +z)를 카메라 규약(x 오른쪽, y 아래, z 멀어짐)으로 바꾸는 건 세 축 부호 반전이라,
웹캠 때처럼 `--mirror-lateral --pick-place`로 쓰면 내 오른쪽 → 로봇 오른쪽, 앞 → 앞, 위 → 위. Studio 캘리브레이션
때 **로봇과 같은 방향을 보고** 서야 축이 맞는다 (글러브만으로는 몸 방향을 모른다). 녹화 샘플을 sink 경로에
재생해 오른쪽 39 cm 이동 → EE Y +24 cm(클램프 25 cm 안), 위로 17 cm → Z 상한 +11 cm에서 멈춤을 확인했다.
`--nero-output`/`--orca-output`을 `real`로 주면 실제 로봇에도 그대로 나가므로(sink가 같다),
실제 핸드의 벌림 방향을 확인하기 전까지는 `sim`으로만 쓴다.

패킷이 안 들어오면 `sudo tcpdump -i any udp port 14043 -c 5`로 도착 여부부터 본다.
포트는 한 프로세스만 쓸 수 있으니 `play.py --live`와 `record.py`를 동시에 돌리지 말 것.

`play.py`는 시작 후 **1초간 손을 쫙 편 상태**를 손 크기 보정에 쓴다 (웹캠과 같은 방식).
`--smoothing 0.8 --max-step-deg 30`이 체감 지연이 적어 쓰고 있는 값이다.

## 리타게팅 (`rokoko/retarget.py`)

`RokokoHandRetargeter(model_path, urdf_path)` — 입력: Rokoko 프레임 dict 또는 (21,3) 배열,
출력: `OrcaJointPositions`(도) 또는 보정 구간 동안 `None`. `joint_array()`로 17개 배열.

1. **좌표계**: Rokoko는 왼손 좌표계(Y up, Z forward). z 부호를 반전해 오른손 좌표계로 만든다.
   녹화로 검증: 반전 없이는 굽힌 손끝이 손등 쪽(canonical z < 0)으로 간다.
2. **굽힘 (mcp/pip × 4)**: 관절 순서가 MediaPipe와 같아서 `orca_teleop`의
   `AdaptiveAnalyticalRetargeter`에 `source="mediapipe"`로 그대로 넣는다. 손목 기준 정규화·자동
   스케일·관절 범위 처리는 리타게터 안에서 한다. 주먹 쥐기/펴기가 깨끗하게 나온다.
3. **벌림 (abd × 4)**: 리타게터 출력을 쓰지 않고 장갑 기하에서 직접 계산(`abduction_from_canonical`).
   장갑의 벌림 신호가 작아(벌림↔붙임에서 손끝 간격 변화 ~2 cm, 약지는 4°) IK 출력으로는 거의
   움직이지 않았다. 각 손가락 첫마디(MCP→PIP) 방향의 손바닥면 내 각도를 `ABD_CALIB`로 선형 매핑.
   주먹을 쥐면(mcp 30→70°) "살짝 모은" 값으로 페이드.
4. **엄지 (cmc/abd/mcp/dip)**: 역시 직접 계산(`thumb_from_canonical`, `THUMB_CALIB`). IK는 엄지 해가
   여러 개라 접었다 펴면 다른 자세에 머무는 이력 현상이 있었고 dip이 하한에 붙어 있었다.
   신호: MCP 굽힘각, IP 굽힘각, 첫마디의 면내 각도(→ cmc), 면 밖 고도각(→ abd).
   장갑에서는 "접기"와 "손바닥 가로지르기"가 거의 같은 신호라 둘을 구분하지 못한다.
5. **후처리**: EMA(`smoothing`) + 프레임당 스텝 제한(`max_step_deg`) + 시뮬 관절 범위 clip.
   `orca_teleop` 설정은 `lp_alpha: 1.0`(스무딩 없음)이라 한 프레임에 30° 넘게 튀는 경우가 있었다.

### 사용자별 보정 (`--calibrate` / `--rokoko-calibrate`)

`ABD_CALIB`·`THUMB_CALIB`의 장갑 쪽 값은 **한 사람(2026-11-16 녹화) 기준**이라 다른 손에서는 "손가락을
다 붙였는데 로봇은 벌어져 있음" 같은 차이가 난다. 그래서 시작 시 안내에 따라 3자세를 취하면 장갑 쪽 값만
그 사람 것으로 바꾸는 `GuidedCalibrator`를 넣었다 (Orca 쪽 목표각은 고정: 벌림 −25/−10/15/28, 붙임
20/8/−8/−25, 엄지 주먹 cmc 33·abd 45·mcp/dip 90). **손가락 굽힘(mcp/pip)도 같이 보정한다**: 리타게터
출력이 손가락마다 범위가 달라(주먹에서 검지 mcp 69·pip 89, 중지 83·82, 약지 63·107, 새끼 56·107) 실물
검지가 끝까지 안 접혔다. 펴기 자세의 출력을 0°, 주먹 자세의 출력을 mcp 90·pip 100으로 보내는 손가락별
선형 보정(`HandCalibration.flex`)을 리타게터 출력 뒤에 건다. 차이가 20° 미만인 관절은 그대로 둔다.

```bash
$PY rokoko/play.py --live --calibrate                       # 단독 뷰어에서
$PY teleop.py --source rokoko --rokoko-calibrate ...        # 텔레옵에서
```

순서: 손 펴고 1초(크기 보정, 기존) → **① 손 펴고 손가락 전부 붙이기 ② 손가락·엄지 최대로 벌리기 ③ 주먹
(엄지도 접기)**. 자세마다 4초 이동(매초 카운트다운) + 2초 측정, 터미널에 `[calib n/3] ...` 안내가 뜬다. 끝나면 중앙값으로
표를 만들어 `rokoko/calib/hand.json`에 저장하고(gitignore), 이후 `play.py`·`teleop.py`·`record_dataset.py`
모두 그 파일을 자동으로 읽는다 (시작 로그 `glove calibration: file ...`). 두 자세의 장갑 값 차이가 2.5°
미만이거나 순서가 어긋난 관절은 기본 상수를 유지하고 로그에 `기본값 유지:`로 알린다 — 엄지 고도각
(`thumb_abd`)은 자세 간 차이가 3° 안팎이라 자주 기본값으로 남는다. 녹화 샘플을 이어 붙인 테스트에서
보정 후 벌림/붙임/주먹이 목표각에 정확히 닿았다 (기본 상수로는 약지 붙임이 −17°로 과했고 검지 주먹이
69/89°에서 멈췄음). 1번 자세에서 손가락을 **곧게** 펴야 그 자세가 0°가 된다.
파일을 지우면 기본 상수로 돌아간다.

## 시뮬 모델 수정

`attach_orca.py`에 `flip_finger_abduction()` 추가, `models/nero_orca.xml` 재생성. 공식 v2 MJCF는
검지·중지·약지·새끼 `*-abd`의 `+`가 검지 쪽으로 움직이는데, 리타게터·orca_core가 쓰는 v1 URDF는
`+`가 새끼 쪽이다(범위도 거울상: 검지 MJCF −25~30 vs config −30~25). 축을 뒤집어 시뮬 표시가
데이터셋 action 기준과 일치하게 했다. 웹캠 텔레옵의 시뮬 표시도 같이 바로잡힌다.
엄지 관절은 두 모델 방향이 같아 그대로.

실물 핸드의 관절 방향은 `orca_joint_test.py`로 관절 하나씩 ±각도를 보내 눈으로 확인한다
(`--joint index_abd --deg 20`: `+`에서 검지 끝이 새끼 쪽이면 정상). 반대면 `config/orcahand_right/config.yaml`의
해당 모터 부호를 뒤집고 `calibration.yaml`의 그 모터 두 값 순서를 바꾼다 (재캘리브레이션 불필요).

## 측정값

| 항목 | 값 |
|---|---|
| 수신율 | 30 Hz, 패킷 ~9.9 KB, 핫스팟에서 0.1 s 넘는 끊김 30 s당 3회 |
| 리타게팅 시간 | 9–15 ms/프레임 (SLSQP) |
| 시뮬 액추에이터 응답 | 90° 스텝에 162 ms (kp=2, 시뮬 고유) |
| 손끝–손목 거리 (펴기/쥐기) | 검지 16.9→5.7 cm, 중지 17.2→5.0, 약지 16.2→4.4, 새끼 14.3→5.6, 엄지 12.2→6.0 |
| 손 크기 자동 보정 비율 | 엄지 1.0, 검지 1.6, 중지 1.5, 약지 1.5, 새끼 1.5 (엄지만 다름 → 직접 계산으로 전환한 이유) |

## 데이터셋 녹화

`orca_teleop/scripts/record_dataset.py`에 `--source rokoko`를 넣었다 (gRPC 인그레스·퍼블리셔 대신
`rokoko_worker` 스레드, 나머지 녹화 루프는 동일). nero_orca 쪽에서는:

```bash
$PY record_dataset.py --source rokoko --fps 15 --episode-end space --num-episodes 5 \
    --urdf-path $ORCAHAND_DESCRIPTION_DIR/v1/models/urdf/orcahand_right.urdf \
    --task "flex fingers, arm at rest" --overwrite      # → datasets/nero-orca-sim-rokoko
SOURCE=rokoko NERO=sim ORCA=sim ./record_pick.sh         # pick-place 설정 + 데이터 카메라
```

`--local`/`--depth`/`--teleop-*`는 글러브에서는 쓰지 않는다. action의 손 17개는 글러브 값, 팔 7개는 홈.

## 실물 핸드

```bash
$PY orca_joint_test.py --joint index_abd --deg 20          # 관절 방향 확인 (토크 켜고 ±20°, 끝나면 0°·토크 끔)
$PY teleop.py --source rokoko --pick-place --nero-output sim --orca-output real \
    --model-path config/orcahand_right/config.yaml          # 글러브 → 실물 손, 팔은 시뮬
$PY teleop.py --source rokoko --pick-place --mirror-lateral --rokoko-arm \
    --nero-output sim --orca-output sim                     # 팔도 글러브 위치로 (시뮬에서 먼저)
SOURCE=rokoko NERO=real ORCA=real SPEED=10 ./record_pick.sh  # 팔까지 (can0 up + External CAN Push 먼저)
```

11/17: 실물 손이 시뮬과 같게 따라옴. 사람 손과의 차이는 사용자별 보정으로 처리.

**손 기울기**: pick 모드는 J2+J4+J7을 `PICK_PITCH`로 묶는다. 원래 135°(손가락 약 45° 아래, 테이블 위에서
내려다보며 집는 자세)였는데 글러브로 앞의 물건을 잡기엔 너무 숙여져 있어 11/17에 **기본값을 90°(플랜지
J7→Orca 손목이 정확히 수평, 손이 앞을 향함)로 바꿨다**. `--pick-pitch-deg`로 바꿀 수 있고(`NeroArmIK(pick_pitch_deg=…)`,
`record_dataset.py`에도 같은 옵션) 135를 주면 예전 자세다. 홈에서 J4 107°, J7 −44°.
팔 작업공간(클램프·반경)은 그대로라 EE 이동 범위는 같고, 글러브 손목 굽힘도 그대로 더해진다.
pick 모드 작업공간은 앞뒤 ±10 cm(반경 0.40~0.675 m 제한으로 넓혀도 ±12), 좌우 ±25, 위아래 −9~+11 cm로 작다.
범위를 보려면 `--pick-place` 없이(±25/30/25 cm, 6자유도) 돌리고, `--rokoko-arm-scale`(기본 1.5)로 배율을 키운다.

### 6자유도 모드 (`--arm-6dof`)

pick 모드는 손 방향을 고정한 3자유도라 사람 팔처럼 보이지 않는다. `--arm-6dof`는 `--pick-place` 없이 쓰는 free
모드의 확장으로, Nero 7관절이 손의 **위치와 방향 변화**를 같이 따라간다 (`NeroArmIK(track_orientation=True)`).

```bash
$PY teleop.py --source rokoko --arm-6dof --mirror-lateral --rokoko-arm --nero-output sim --orca-output sim
```

- 방향 입력은 Rokoko `rightHand`의 전역 회전(IMU 기반, 위치 추정과 달리 믿을 만함; 샘플에서 손 로컬 −y가
  손목→중지 MCP 방향과 내적 −1.00). `rokoko.retarget.hand_rotation_matrix` → `sink.update_hand_pose(xyz, pitch, rot)`
  → `WristPosition.hand_rot`. 트래커가 오면 위치(그리고 원하면 방향)만 갈아끼운다.
- 첫 샘플을 기준으로 한 **상대 회전**만 쓴다: ΔR = R·R₀ᵀ(Rokoko 좌표)를 카메라→로봇 행렬 T로 켤레(T ΔR Tᵀ)해
  기준 시점의 EE 방향에 곱한다. 절대 방향을 안 쓰므로 글러브 본의 축 정의·왼손 좌표계 문제가 없다.
  회전벡터에 One Euro 필터, 180°/s 속도 제한.
- IK: 6×7 자코비안(위치 + `ORIENT_WEIGHT` 0.2 m/rad × 회전) 감쇠 최소제곱, 여유 자유도는 홈 자세(플랜지 수평,
  pick 홈과 같음)로. 정적 목표에 위치 0.0 cm·회전 0.2° 수렴(IK 해 기준). 작업공간·속도는 free 모드(±25/30/25 cm, 0.8 m/s).
- 시뮬에서 보이는 지연(녹화 샘플 재생: 방향 중앙값 18°, 위치 4.5 cm)은 **시뮬 Nero 서보의 토크 제한**이다. 필터+속도제한
  1°, IK 0.1°, 서보 19°로 분해됐고, kp를 400→4000으로 올려도 18°라 게인이 아니라 `forcerange`(URDF effort, J2 24 Nm 등)에
  막힌다. 정지 목표에서는 3°/2 cm(중력 처짐 J2 1.7°, Orca 손목 액추에이터 kp 2라 손목 7° 처짐). 실물은 Nero 자체 제어기와
  `--nero-speed`가 결정한다.
- 실물에서는 방향까지 따라가면 자기 충돌·관절 한계 위험이 있으므로 시뮬에서 충분히 본 뒤 `--nero-speed 10`으로.
  `record_dataset.py`에는 아직 안 넣었다.

## 다음 단계

- 팔 손목 위치: VIVE Ultimate Tracker (글러브의 `rightHand` 위치는 슈트 없이 추정값이라 쓰지 않음).
- 팔: Rokoko `rightHand` 위치(전신 좌표계, m)를 `NeroArmIK`의 손목 목표로 쓰는 것도 가능하지만
  글러브만으로는 위치 드리프트가 있어 Quest 3/트래커 계획과 비교해 결정.
