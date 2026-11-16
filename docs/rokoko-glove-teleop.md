# Rokoko Smartgloves → Orca hand (sim)

2026-11-16. Rokoko Smartgloves(오른손) 실시간 스트림을 리눅스 노트북에서 받아 Orca 핸드 관절
목표값으로 바꾸고 MuJoCo에서 확인한 기록. **실제 로봇 제어 경로(`teleop.py`,
`combined_sink.py`, `record_pick.sh`)에는 아직 연결하지 않았다.**

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
pick 모드가 아니면 손목은 0° 그대로. 팔 위치 입력은 VIVE Ultimate Tracker로 할 계획이라 글러브 위치는 쓰지 않는다.
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

`ABD_CALIB`·`THUMB_CALIB`는 **한 사람(2026-11-16 녹화) 기준 상수**다. 다른 사람이 끼거나 장갑을
재보정하면 `rokoko/samples/abd_test.npz`, `thumb_test.npz`와 같은 동작을 다시 녹화해 값을 갱신한다
(시작 시 벌림/붙임·엄지 동작 2초씩으로 자동 보정하는 것은 다음 단계).

## 시뮬 모델 수정

`attach_orca.py`에 `flip_finger_abduction()` 추가, `models/nero_orca.xml` 재생성. 공식 v2 MJCF는
검지·중지·약지·새끼 `*-abd`의 `+`가 검지 쪽으로 움직이는데, 리타게터·orca_core가 쓰는 v1 URDF는
`+`가 새끼 쪽이다(범위도 거울상: 검지 MJCF −25~30 vs config −30~25). 축을 뒤집어 시뮬 표시가
데이터셋 action 기준과 일치하게 했다. 웹캠 텔레옵의 시뮬 표시도 같이 바로잡힌다.
엄지 관절은 두 모델 방향이 같아 그대로.

**실제 Orca 핸드의 벌림 방향은 아직 확인하지 않았다.** 하드웨어로 돌리기 전에 `index_abd +20`을
보내 손끝이 새끼 쪽으로 가는지 확인할 것.

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

## 다음 단계

- 팔 손목 위치: VIVE Ultimate Tracker (글러브의 `rightHand` 위치는 슈트 없이 추정값이라 쓰지 않음).
- 실제 핸드 벌림 방향 확인 후 하드웨어 연결.
- 팔: Rokoko `rightHand` 위치(전신 좌표계, m)를 `NeroArmIK`의 손목 목표로 쓰는 것도 가능하지만
  글러브만으로는 위치 드리프트가 있어 Quest 3/트래커 계획과 비교해 결정.
