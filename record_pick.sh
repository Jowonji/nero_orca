#!/usr/bin/env bash
# Pick-and-place recording on the real Nero+Orca (docs/native-linux-recording-setup.md §10).
#
#   ./record_pick.sh                       # arm real 20%, hand real, 1 episode, overwrite
#   NERO=shadow ORCA=real ./record_pick.sh # hand only
#   NAME=pick-cube TASK="pick the cube and place it left of the tray" EPISODES=20 KEEP=1 ./record_pick.sh
#
# Env: NERO / ORCA (sim|shadow|real), SPEED (Nero %), NAME (dataset name), TASK, EPISODES,
# KEEP=1 to append instead of --overwrite. The data camera is found by its serial, so the
# /dev/video number may change between replugs.
set -euo pipefail
cd "$(dirname "$0")"

DATA_CAM_SERIAL=CP0H953001AR     # recording (observation.images.data) Orbbec
TELEOP_CAM_SERIAL=CP0H95300129   # hand-tracking Orbbec (RGB-D)

NERO=${NERO:-real}
ORCA=${ORCA:-real}
SPEED=${SPEED:-20}
NAME=${NAME:-nero-orca-pick-real-test}
TASK=${TASK:-pick real test}
EPISODES=${EPISODES:-1}

data_index=""
for dev in /dev/video*; do
    v4l2-ctl -d "$dev" --list-formats 2>/dev/null | grep -q MJPG || continue
    serial=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_SERIAL_SHORT=//p')
    if [[ "$serial" == "$DATA_CAM_SERIAL" ]]; then data_index=${dev#/dev/video}; break; fi
done
if [[ -z "$data_index" ]]; then
    echo "data camera $DATA_CAM_SERIAL not found (replug it?)" >&2
    exit 1
fi
if ! ip link show can0 2>/dev/null | grep -q "state UP" && [[ "$NERO" != sim ]]; then
    echo "can0 is not up: sudo ip link set can0 up type can bitrate 1000000, then External CAN Push" >&2
    exit 1
fi
echo "data camera /dev/video$data_index, nero=$NERO (speed $SPEED%), orca=$ORCA, dataset=$NAME"

overwrite=(--overwrite)
[[ "${KEEP:-0}" == 1 ]] && overwrite=()

../orca_teleop/.venv/bin/python record_dataset.py \
    --local --source mediapipe --show-video \
    --depth orbbec \
    --teleop-orbbec-serial "$TELEOP_CAM_SERIAL" \
    --teleop-orbbec-laser-level 3 \
    --mirror-lateral --pick-place \
    --camera "data:$data_index" \
    --nero-output "$NERO" --nero-speed "$SPEED" \
    --orca-output "$ORCA" \
    --model-path config/orcahand_right/config.yaml \
    --urdf-path ../orcahand_description/v1/models/urdf/orcahand_right.urdf \
    --repo-id "local/$NAME" \
    --root "datasets/$NAME" \
    --task "$TASK" \
    --fps 15 --num-episodes "$EPISODES" --episode-end space \
    "${overwrite[@]}" 2>&1 | tee record_real.log
