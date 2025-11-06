#!/usr/bin/env bash

LIN=${1:-2.0}
ANG=${2:-0.3}
DUR=${3:-5}
OUT=${4:-twist_log.txt}

echo "Logging /tb3/ground_truth/twist to $OUT ..."

echo "Setting to 0"

timeout 5s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1

(
    ros2 topic echo /tb3/ground_truth/twist --field twist.angular.z \
    | while read -r VAL; do 
        printf "%s,%s\n" "$(date +%s.%N)" "$VAL" >> "$OUT"
    done
) & LOG_PID=$!

echo "Setting speed"

timeout 5s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${LIN}}, angular: {z: ${ANG}}}" >/dev/null 2>&1

pkill -f "ros2 topic echo /tb3/ground_truth/twist" 

echo "Done. Output in $OUT"