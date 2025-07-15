#!/usr/bin/env python3

import os
import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import math

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# for joycon
from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot, xarm6_lerobot_FK, xarm6_lerobot_IK
# from joyconrobotics.joyconrobotics import JoyconRobotics
from joyconrobotics.joyconrobotics import JoyconRobotics


######################## ros  #############################
import rospy
from std_msgs.msg import Float64MultiArray

# 初始化 ROS 节点（放在 arm 初始化前）
rospy.init_node('xarm_joycon_node', anonymous=True)
pub_action = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=10)
pub_state = rospy.Publisher('/robot_state', Float64MultiArray, queue_size=10)

######################################
np.set_printoptions(linewidth=200)
os.environ["MUJOCO_GL"] = "egl"

from xarm.wrapper import XArmAPI  
arm = XArmAPI('192.168.1.222')
arm.motion_enable(enable=True) 
arm.set_gripper_enable(enable=True)
arm.set_mode(6)  
arm.set_state(0)  

JOINT_NAMES = ["robot0:shoulder_pan_joint","robot0:shoulder_lift_joint","robot0:elbow_flex_joint",
               "robot0:forearm_roll_joint","robot0:wrist_flex_joint","robot0:wrist_roll_joint"]

xml_path = "./examples/scene_xarm6.xml"
mjmodel = mujoco.MjModel.from_xml_path(xml_path)
qpos_indices = np.array([mjmodel.jnt_qposadr[mjmodel.joint(name).id] for name in JOINT_NAMES])
mjdata = mujoco.MjData(mjmodel)

JOINT_INCREMENT = 0.01  
POSITION_INSERMENT = 0.002

robot = get_robot('xarm6')



import numpy as np

# 角度限制（单位：弧度）
JOINT_LIMITS_RAD = {
    0: (np.deg2rad(-360), np.deg2rad(360)),    # J1
    1: (np.deg2rad(-117), np.deg2rad(116)),    # J2
    2: (np.deg2rad(-219), np.deg2rad(10)),     # J3
    3: (np.deg2rad(100), np.deg2rad(283)),    # J4
    4: (np.deg2rad(-20), np.deg2rad(180)),     # J5
    5: (np.deg2rad(-100), np.deg2rad(80))     # J6
}

def unwrap_and_clamp_angles_rad(curr_joint_angles, target_joint_angles):
    """
    消除目标弧度的跳变（unwrap），并限制在合法的关节范围内（clamp）。
    如果某个关节角度被 clip，会打印提示信息。
    """
    curr_joint_angles = np.array(curr_joint_angles)
    target_joint_angles = np.array(target_joint_angles)

    # unwrap：避免 π 跳变
    delta = target_joint_angles - curr_joint_angles
    delta_unwrapped = (delta + np.pi) % (2 * np.pi) - np.pi
    target_unwrapped = curr_joint_angles + delta_unwrapped

    clamped = []
    for i in range(6):
        lower, upper = JOINT_LIMITS_RAD[i]
        val = target_unwrapped[i]
        if val < lower or val > upper:
            print(
                f"[⚠️ Clip] Joint J{i+1}: "
                f"{val:.3f} rad ({np.rad2deg(val):.2f}°) "
                f"was out of limit [{lower:.3f}, {upper:.3f}] "
                f"({np.rad2deg(lower):.2f}°, {np.rad2deg(upper):.2f}°)"
            )
        clamped.append(np.clip(val, lower, upper))

    return clamped



# init_qpos = np.array([14.1, -8, -24.7, 196.9, 62.3, -8.8])
init_qpos = np.array([4.3, 15.5, -9.4, 182.6, 100.4 ,0.3])
# init_qpos = np.array([7, 19.7, -20.2, 182.1, 88.1, -1.7])
init_qpos = np.radians(init_qpos)
# print("init_qpos", init_qpos)
arm.set_servo_angle(angle=init_qpos,speed=8,is_radian=True)

mjdata.qpos[qpos_indices] = init_qpos

target_qpos = init_qpos.copy() 
init_gpos = xarm6_lerobot_FK(init_qpos,arm)
target_gpos = init_gpos.copy() 

lock = threading.Lock()
target_gpos_last = init_gpos.copy()
last_time_qpos = init_qpos.copy()
direction_data_r = [[], [], []]  
# print("init_gpos", init_gpos)
offset_position_m = init_gpos[0:3]
offset_euler_rad = np.radians(init_gpos[3:]) 

control_glimit = [
    [-0.665, -0.665, -0.380, -3.1, -3.1, -3.1],   # 下限（米 + 弧度）
    [ 0.665,  0.665,  0.940,  3.1,  3.1,  3.1],   # 上限
]


joyconrobotics_right = JoyconRobotics(
    device="right", 
    horizontal_stick_mode='yaw_diff', 
    arm=arm,
    close_y=False, 
    limit_dof=True, 
    glimit=control_glimit,
    offset_position_m=offset_position_m, 
    offset_euler_rad=offset_euler_rad,
    common_rad=False,
    lerobot=True,
    pitch_down_double=True,
)

initial_pitch = joyconrobotics_right.get_control()[0][4]

# time.sleep(1.0)  # 等待 Joycon 完成初始校准
# joyconrobotics_right.set_position = init_gpos[0:3]
# joyconrobotics_right.set_euler = init_gpos[3:]
# print("🧪 Offset Euler:", init_gpos[3:])
# print("🧪 Joycon raw pose:", joyconrobotics_right.get_control()[0][3:])


# ---------- 主循环 ----------
control_frequency = 30
time_interval = 1 / control_frequency

t = 0
try:
    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < 1000:
            step_start = time.time()
            if t == 0:
                mjdata.qpos[qpos_indices] = init_qpos
                mujoco.mj_step(mjmodel, mjdata)
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(mjdata.time % 2)
                viewer.sync()

            t += 1
            target_pose, gripper_state_r, _ = joyconrobotics_right.get_control()

            # print("gripper_state_r", gripper_state_r)


            # 从 Joycon 获取末端目标位姿
            x_r, y_r, z_r, roll_r, pitch_r, yaw_r = target_pose
            current_pitch = pitch_r
            delta_pitch = current_pitch - initial_pitch
            corrected_pitch = initial_pitch - delta_pitch

            # print("initial_pitch", initial_pitch)
            # print("current_pitch", current_pitch)
            # print("delta_pitch", delta_pitch)
            # print("corrected_pitch", corrected_pitch)

            right_target_gpos = np.array([x_r, y_r, z_r, roll_r, corrected_pitch, yaw_r])

            # print("🤖 robot_target_pose", right_target_gpos)

            # 逆运动学求解目标关节角
            qpos_inv, IK_success = xarm6_lerobot_IK(mjdata.qpos[qpos_indices], right_target_gpos,arm)

            # print("qpos_inv", qpos_inv)

            if IK_success:
                current_qpos = mjdata.qpos[qpos_indices]
                target_qpos = qpos_inv
                last_time_qpos = target_qpos

                control_pos = target_qpos.copy()
                # control_pos[3] += 6.2831852
                #tackle angles jump
                _, curr_angles_rad = arm.get_servo_angle(is_radian=True)

                # print("curr_angles_rad", np.rad2deg(curr_angles_rad[:-1]))             
                # print("control_pos", np.rad2deg(control_pos))
                command_angles_rad = unwrap_and_clamp_angles_rad(curr_angles_rad[:-1], control_pos)
                # print("command_angles_rad", np.rad2deg(command_angles_rad))
                ########################## ROS ############################
                # 发布 action 和 state（去掉最后一个 gripper 关节）
                _, curr_gripper_state = arm.get_gripper_position()
                # combine curr_angles_rad and curr_gripper_state
                curr_state_msg = np.append(np.rad2deg(curr_angles_rad[:-1]), curr_gripper_state)
                state_msg = Float64MultiArray(data=curr_state_msg)

                if gripper_state_r == 1:
                    curr_action_msg = np.append(np.rad2deg(command_angles_rad), 730)
                else:
                    curr_action_msg = np.append(np.rad2deg(command_angles_rad), 50)

                action_msg = Float64MultiArray(data=curr_action_msg)
                pub_state.publish(state_msg)
                pub_action.publish(action_msg)
                # print("publish state", state_msg.data)
                # print("publish action", action_msg.data)
                ############################################################


                # print("robot target_qpos", control_pos)
                # mjdata.qpos[qpos_indices] = control_pos
                mjdata.qpos[qpos_indices] = command_angles_rad
                mujoco.mj_step(mjmodel, mjdata)
                # print("robot target_gpos",xarm6_lerobot_FK(control_pos,arm))

                ###### START control real robot #######
                arm.set_servo_angle(angle=command_angles_rad,speed=80,is_radian=True)

                if gripper_state_r == 1:
                    # print("gripper open")
                    arm.set_gripper_position(pos=730, wait=False)
                else:
                    # print("gripper close")
                    arm.set_gripper_position(pos=50, wait=False)
                ###### END control real robot #######


                # print("xarm6_real_qpos",arm.get_servo_angle(is_radian=True))

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(mjdata.time % 2)
                viewer.sync()

                target_gpos_last = right_target_gpos.copy()
            else:
                right_target_gpos = target_gpos_last.copy()
                joyconrobotics_right.set_position = right_target_gpos[0:3]
                joyconrobotics_right.set_euler = init_gpos[3:]

            # time_until_next_step = mjmodel.opt.timestep - (time.time() - step_start)
            time_until_next_step = time_interval - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            time.sleep(0.001)

except KeyboardInterrupt:
    viewer.close()
