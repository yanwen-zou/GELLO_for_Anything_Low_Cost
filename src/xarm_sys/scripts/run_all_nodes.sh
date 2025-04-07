#!/bin/bash

# === 设置 ROS 环境 ===
source /opt/ros/noetic/setup.bash
source /root/xarm_ws/devel/setup.bash

echo "[INFO] Starting all nodes..."

# === 启动每个节点 ===
rosrun xarm_sys cam_pub.py &
PID1=$!
echo "[INFO] cam_pub.py started with PID $PID1"

rosrun xarm_sys xarm_pub.py &
PID2=$!
echo "[INFO] xarm_pub.py started with PID $PID2"

rosrun xarm_sys episode_recorder.py &
PID3=$!
echo "[INFO] episode_recorder.py started with PID $PID3"

# === 设置 Ctrl+C 时的清理逻辑 ===
trap "echo '[INFO] Ctrl+C received. Shutting down all nodes...'; kill $PID1 $PID2 $PID3; exit" SIGINT

# === 等待所有子进程结束 ===
wait
