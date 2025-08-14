#!/bin/bash

# === 设置 ROS 环境 ===
source /opt/ros/noetic/setup.bash

# 脚本所在的目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "$SCRIPT_DIR/../devel/setup.bash"

echo "[INFO] ROS 环境已加载，开始启动节点..."


# === 启动 servo_reader.py ===
rosrun UArm servo_reader.py &
PID1=$!
echo "[INFO] servo_reader.py started with PID $PID1"

# === 启动遥操作控制器 ===
rosrun UArm servo2arm.py &
PID2=$!
echo "[INFO] servo2arm.py started with PID $PID2"





# === 设置 Ctrl+C 时的清理逻辑 ===
trap "echo '[INFO] Ctrl+C received. Shutting down all nodes...'; kill $PID1 $PID2; exit" SIGINT

# === 等待所有子进程结束 ===
wait
