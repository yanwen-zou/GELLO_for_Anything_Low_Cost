#!/bin/bash

# 捕获 Ctrl+C 信号并停止所有后台进程
trap "kill $PID1 $PID2 $PID3" SIGINT

# === 设置 ROS 环境变量 ===
echo "Setting up ROS environment..."
source ~/ros_ws/devel/setup.bash  # 确保环境变量加载


# === 启动 lerobot_joycon_gpos_xarm6_new.py ===
echo "Starting lerobot_joycon_gpos_xarm6_new.py..."
rosrun xarm_sys scripts/xarm_joycon/examples/lerobot_joycon_gpos_xarm6_new.py &
PID1=$!
echo "[INFO] lerobot_joycon_gpos_xarm6_new.py started with PID $PID1"
sleep 2  # 等待 JoyCon 控制启动

# === 启动 cam_pub.py ===
echo "Starting cam_pub.py..."
rosrun xarm_sys scripts/xarm_joycon/cam_pub.py &
PID2=$!
echo "[INFO] cam_pub.py started with PID $PID2"
sleep 2  # 等待 cam_pub 启动

# === 启动 episode_recorder.py ===
echo "Starting episode_recorder.py..."
rosrun xarm_sys scripts/xarm_joycon/episode_recorder.py &
PID3=$!
echo "[INFO] episode_recorder.py started with PID $PID3"

# 等待所有后台进程完成
wait $PID1
wait $PID2
wait $PID3

# 完成
echo "[INFO] All nodes are running."
