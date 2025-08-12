#!/bin/bash
# 运行servo2Dobot.py脚本

source /opt/ros/noetic/setup.bash
source /home/onestar/teleop_Bestman/devel/setup.bash

echo "servo2Dobot.py..."
rosrun xarm_sys servo2Dobot.py &
PID1=$!


# 运行servo_reader.py脚本
echo "servo_reader.py..."
rosrun xarm_sys servo_reader.py &
PID2=$!


# === 设置 Ctrl+C 时的清理逻辑 ===
trap "echo '[INFO] Ctrl+C received. Shutting down all nodes...'; kill $PID1 $PID2; exit" SIGINT

# === 等待所有子进程结束 ===
wait