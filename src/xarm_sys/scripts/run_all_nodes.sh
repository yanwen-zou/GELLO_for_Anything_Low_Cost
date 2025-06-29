#!/bin/bash

# === 设置 ROS 环境 ===
source /opt/ros/noetic/setup.bash
source /root/xarm_ws/Xarm_ros_deploy/devel/setup.bash

echo "[INFO] ROS 环境已加载，开始启动节点..."

# === 启动 cam_pub.py ===
rosrun xarm_sys cam_pub.py &
PID1=$!
echo "[INFO] cam_pub.py started with PID $PID1"

# === 启动 episode_recorder.py ===
rosrun xarm_sys episode_recorder.py &
PID2=$!
echo "[INFO] episode_recorder.py started with PID $PID2"

# === 启动 Joycon 遥操作控制器（使用 python3）===
python3 /root/xarm_ws/src/xarm_sys/scripts/Gello/servo2arm.py &
PID3=$!
echo "[INFO] lerobot_joycon_gpos_xarm6_new.py started with PID $PID3"

# === 设置 Ctrl+C 时的清理逻辑 ===
trap "echo '[INFO] Ctrl+C received. Shutting down all nodes...'; kill $PID1 $PID2 $PID3; exit" SIGINT

# === 等待所有子进程结束 ===
wait
