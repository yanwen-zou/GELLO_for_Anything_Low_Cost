# code by Boxjod LinCC111 2025.1.13 Box2AI-Robotics copyright 盒桥智能 版权所有

import os
import mujoco
import mujoco.viewer
import numpy as np
import time
import threading

from lerobot_kinematics import lerobot_IK, lerobot_FK, get_robot, xarm6_lerobot_FK ,xarm6_lerobot_IK
from joyconrobotics import JoyconRobotics
import math

# from xarm.wrapper import XArmAPI  
# arm = XArmAPI('192.168.1.222')
# arm.motion_enable(enable=False)  
# arm.set_mode(0)  
# arm.set_state(0)  

np.set_printoptions(linewidth=200)
os.environ["MUJOCO_GL"] = "egl"

JOINT_NAMES = ["robot0:shoulder_pan_joint","robot0:shoulder_lift_joint","robot0:elbow_flex_joint","robot0:forearm_roll_joint","robot0:wrist_flex_joint","robot0:wrist_roll_joint"]

xml_path = "./examples/scene_xarm6.xml"
mjmodel = mujoco.MjModel.from_xml_path(xml_path)
qpos_indices = np.array([mjmodel.jnt_qposadr[mjmodel.joint(name).id] for name in JOINT_NAMES])
mjdata = mujoco.MjData(mjmodel)
robot = get_robot('xarm6')
# ✅ 初始位姿定义
init_qpos = np.array([16.4, -30.2, -52.4, -5.4, 84, 12.5])
init_qpos = np.radians(init_qpos)
mjdata.qpos[qpos_indices] = init_qpos  # ✅ 初始化仿真器的 qpos
mujoco.mj_forward(mjmodel, mjdata)     # ✅ 推荐使用 mj_forward 而不是 mj_step 初始化状态



control_glimit = [[-10,-10,-10, -10., -10, -10], 
                  [ 10,  10,  10,  10,   10,  10]]

target_qpos = init_qpos.copy() 
init_gpos = np.array([0.3245, 0.0743, 0.4585, -3.069, 0, 0])
print("init_gpos", init_gpos)
target_gpos = init_gpos.copy() 

lock = threading.Lock()
target_gpos_last = init_gpos.copy()
direction_data_r = [[], [], []]  

offset_position_m = init_gpos[0:3]
joyconrobotics_right = JoyconRobotics(device="right", 
                                      horizontal_stick_mode='yaw_diff', 
                                      close_y=True, 
                                      limit_dof=True, 
                                      glimit=control_glimit,
                                      offset_position_m=offset_position_m, 
                                      common_rad=False,
                                      lerobot=True,
                                      pitch_down_double=True)

control_frequency = 20
time_interval = 1 / control_frequency

last_time_qpos = init_qpos
t = 0

try:
    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < 1000:
            step_start = time.time()
            t += 1

            target_pose, gripper_state_r, _ = joyconrobotics_right.get_control()
            print("手柄当前的末端target_pose:", [f"{x:.3f}" for x in target_pose])

            for i in range(6):
                target_pose[i] = max(min(target_pose[i], control_glimit[1][i]), control_glimit[0][i])

            x_r, y_r, z_r, roll_r, pitch_r, yaw_r = target_pose

            right_target_gpos = np.array([x_r, y_r, z_r, roll_r, pitch_r, yaw_r])
            print("更正标定后的末端right_target_gpos", right_target_gpos)

            print("环境中上一时刻关节角度mjdata.qpos[qpos_indices]", mjdata.qpos[qpos_indices])

            qpos_inv, IK_success = xarm6_lerobot_IK(mjdata.qpos[qpos_indices], right_target_gpos, robot=robot)
            print("ik计算出的关节角度qpos_inv", qpos_inv)

            if IK_success:
                # ✅ 多关节敏感度控制
                joint_diff_thresholds = [0.3, 0.3, 0.3, 0.3, 0.35, 0.3]
                sum_threshold = 0.4
                sum = 0
                current_qpos = mjdata.qpos[qpos_indices]

                exceed_threshold = False
                for i in range(len(qpos_inv)):
                    diff = abs(qpos_inv[i] - current_qpos[i])
                    sum += diff
                    print("diff", diff)
                    if sum > sum_threshold:
                        exceed_threshold = True
                        break
                    # if diff > joint_diff_thresholds[i]:
                    #     print(f"关节 {i} 超出阈值: 当前差值 {diff:.3f} > 阈值 {joint_diff_thresholds[i]}")
                    #     exceed_threshold = True
                    #     break

                if not exceed_threshold:
                    print("继承")
                    target_qpos = last_time_qpos
                else:
                    print("进一步")
                    target_qpos = qpos_inv

                last_time_qpos = target_qpos
                print("下一时刻的qpos", target_qpos)
                mjdata.qpos[qpos_indices] = target_qpos
                # arm.set_servo_angle(angle=target_qpos,speed=300,is_radian=True)

                mujoco.mj_step(mjmodel, mjdata)

                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(mjdata.time % 2)
                viewer.sync()

                target_gpos_last = right_target_gpos.copy()
            else:
                right_target_gpos = target_gpos_last.copy()
                joyconrobotics_right.set_position = right_target_gpos[0:3]

            time_until_next_step = time_interval - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            time.sleep(0.01)

except KeyboardInterrupt:
    viewer.close()
