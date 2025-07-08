#!/bin/bash

# === 设置 ROS 环境 ===
source /opt/ros/noetic/setup.bash
source /home/zhaobo/code/teleop/devel/setup.bash

echo "[INFO] ROS 环境已加载，开始启动节点..."

# === 启动 cam_pub.py ===
rosrun xarm_sys cam_pub.py &
PID1=$!
echo "[INFO] cam_pub.py started with PID $PID1"

rosrun xarm_sys xarm_pub.py &
PID2=$!
echo "[INFO] xarm_pub.py started with PID $pid2"

# === 启动 servo_reader.py ===
python3 /home/zhaobo/code/teleop/src/xarm_sys/scripts/Gello/servo_reader.py &
PID3=$!
echo "[INFO] servo_reader.py started with PID $PID3"


# === 启动 Joycon 遥操作控制器（使用 python3）===
python3 /home/zhaobo/code/teleop/src/xarm_sys/scripts/Gello/servo2arm.py &
PID4=$!
echo "[INFO] servo2arm.py started with PID $PID4"

# === 启动 episode_recorder.py ===
rosrun xarm_sys episode_recorder.py &
PID5=$!
echo "[INFO] episode_recorder.py started with PID $PID5"



# === 设置 Ctrl+C 时的清理逻辑 ===
trap "echo '[INFO] Ctrl+C received. Shutting down all nodes...'; kill $PID1 $PID2 $PID3 $PID4 $PID5; exit" SIGINT

# === 等待所有子进程结束 ===
wait
