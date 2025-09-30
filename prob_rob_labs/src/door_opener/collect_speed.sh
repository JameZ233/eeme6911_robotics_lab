#!/usr/bin/env bash

SPEED=$1
OUTFILE="speed_${SPEED}.txt"

echo "Running speed test"

ros2 launch prob_rob_labs door_opener_launch.py forward_speed:=${SPEED} \
	> "${OUTFILE}" & SIM_PID=$!


ros2 topic echo /odom --field twist.twist.linear.x > "${OUTFILE}"

kill $SIM_PID
wait $SIM_PID 2>/dev/null

echo "DONE."




