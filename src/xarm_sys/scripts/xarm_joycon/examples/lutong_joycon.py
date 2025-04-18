import time
import math
from threading import Thread, Event
from xarm.wrapper import XArmAPI
from joyconrobotics import JoyconRobotics
from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot, xarm6_lerobot_FK ,xarm6_lerobot_IK
import numpy as np
# 初始化 Joy-Con
try:
    joycon = JoyconRobotics("right")
except:
    joycon = JoyconRobotics("left")

# 初始化 xArm6
arm = XArmAPI('192.168.1.222')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)

# 初始末端位姿（单位：mm, 度）
home_pose = [300,100,434,180,0,0]

cur_pose = home_pose.copy()

# 控制频率（秒）
dt = 0.001

# 停止标志
stop_event = Event()

# 程序启动时记录 Joy-Con 的初始位置
initial_pose = joycon.get_control()[0]  # 获取初始姿态向量

def control_robot():
    print("开始遥操作控制 xArm6（按 X 按钮退出）")
    while not stop_event.is_set():
        pose, gripper, control_button = joycon.get_control()

        # 位置偏移（单位从米 → 毫米）
        dx = (pose[0] - initial_pose[0]) * 1000
        dy = (pose[1] - initial_pose[1]) * 1000
        dz = (pose[2] - initial_pose[2]) * 1000

        x = home_pose[0] + dx
        y = home_pose[1] + dy
        z = home_pose[2] + dz

        # 姿态控制（原始数据为弧度，转换为度）
        roll  = math.degrees(pose[3]) + home_pose[3]
        pitch = math.degrees(pose[4]) + home_pose[4]
        yaw   = math.degrees(pose[5]) + home_pose[5]

        cur_pose[:] = [x, y, z, roll, pitch, yaw]
        
        # arm.set_position(*cur_pose, speed=200, wait=False, radius=5)
        
        #使用qpos控制
        cur_q_pose = cur_pose.copy()

        cur_q_pose[0] = cur_pose[0]/1000
        cur_q_pose[1] = cur_pose[1]/1000
        cur_q_pose[2] = cur_pose[2]/1000
        cur_q_pose[3] = np.radians(cur_pose[3])
        cur_q_pose[4] = np.radians(cur_pose[4])
        cur_q_pose[5] = np.radians(cur_pose[5])
        

        print("当前姿态:", [round(p, 2) for p in cur_pose])

        # 按 X 键退出
        if joycon.button.get_button_x() == 1:
            print("检测到 X 按钮，遥操作结束")
            stop_event.set()
            break

        time.sleep(dt)

    arm.motion_enable(False)
    print("控制线程结束，不再发送指令")

def main():
    robot_thread = Thread(target=control_robot)
    robot_thread.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.01)
    except KeyboardInterrupt:
        stop_event.set()

    robot_thread.join()
    arm.disconnect()
    print("程序已退出")

if __name__ == "__main__":
    main()
