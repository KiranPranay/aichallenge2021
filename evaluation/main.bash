#!/bin/bash
source /opt/AutowareAuto/setup.bash
source /opt/aichallenge_ws/install/setup.bash

set -u

export OUTPUT_BAG='/output/all.bag'
export OUTPUT_LOG='/output/log.txt'
export OUTPUT_SCORE='/output/score.json'

ros2 bag record -a -o $OUTPUT_BAG &
PID_ROSBAG=$!

lgsvl_bridge &
PID_BRIDGE=$!

sleep 2

{
  ros2 launch aichallenge_launch aichallenge.launch.py &
  PID_LAUNCH=$!
} &> $OUTPUT_LOG

sleep 10

cd /scenario
timeout 900 python3 scenario.train.py --vehicle_id=$LG_VEHICLE_ID || true

sleep 1

pkill -INT -P $PID_LAUNCH || true
wait $PID_LAUNCH || true
kill -INT $PID_ROSBAG || true
wait $PID_ROSBAG || true
