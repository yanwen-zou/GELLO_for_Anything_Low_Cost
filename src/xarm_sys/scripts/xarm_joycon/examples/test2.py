import numpy as np
from scipy.spatial.transform import Rotation as R
from spatialmath.base import r2t
from lerobot_kinematics import xarm6_lerobot_FK, get_robot

# Step 1: 从真实机械臂界面读取末端位姿（单位：mm 和 角度）
real_xyz_mm = [187.2,-18.8,62.3]             # mm
real_rpy_deg = [-167.7,21.9,35.5]                    # deg

# Step 2: 单位转换
xyz_m = np.array(real_xyz_mm) / 1000.0        # mm → m
rpy_rad = np.radians(real_rpy_deg)           # deg → rad

# Step 3: 构造 target_pose
target_pose = np.concatenate([xyz_m, rpy_rad])

# Step 4: 使用实机角度 [0, 0, 0, 0, 0, 0]
real_qpos_deg = [-11.6,14.3,-8.2,-22.1,19.3,-22.2]
qpos_rad = np.radians(real_qpos_deg)
robot = get_robot("xarm6")

# Step 5: 正向运动学
fk_pose = xarm6_lerobot_FK(qpos_rad, robot=robot)
print("FK Pose from real_qpos (m, rad):", fk_pose)
print("Target Pose (m, rad):", target_pose)

# Step 6: 位姿差值（位置 + 姿态）
pos_error = np.linalg.norm(fk_pose[:3] - target_pose[:3])
print("Position error:", pos_error)

# 计算角度差（更稳定的方式）
fk_r = R.from_euler('xyz', fk_pose[3:]).as_matrix()
target_r = R.from_euler('xyz', target_pose[3:]).as_matrix()
R_rel = fk_r.T @ target_r
angle_diff = np.arccos((np.trace(R_rel) - 1) / 2)
print(f"Rotation error (angle diff in rad): {angle_diff}")
print(f"Rotation error (angle diff in deg): {np.degrees(angle_diff)}")
