#!/usr/bin/env bash

set -euo pipefail

TOPIC="/door_open"
MSG_TYPE="std_msg/msg/Empty"
JOINT_TOPIC="/joint_states"
JOINT_NAME="joint_position"
N_TRIALS=30
SLEEP_BETWEEN="1.0"
OUT_TXT="./door_open_publish_log.txt"

echo "Publishing $N_TRIALS empty message"

for ((i=1; i<=N_TRIALS; i++)); do 
    UNIX_NS="$(date +%s%N)"
    ros2 topic pub --once "$TOPIC" "$MSG_TYPE"  || \
    echo "Warn: Fail to publish"

    JOINT_POS="$(ros2 topic echo "$JOINT_TOPIC" --once | \
                grep -A1 "name:" | grep -A1 "$JOINT_NAME" | tail -n1 | awk '{print $2}')"

    echo "$i, $UNIX_NS, $JOINT_POS " >> "$OUT_TXT"
    echo "--end of trial--" >> "$OUT_TXT"

    sleep "$SLEEP_BETWEEN"

done 

echo "Done."