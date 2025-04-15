import numpy as np
from scipy.spatial.transform import Rotation as R
from lerobot_kinematics import xarm6_lerobot_IK, get_robot

# ===== 1. 真实末端位姿（位置：mm，姿态：deg） =====
real_xyz_mm = [0.3609, 0.0252,0.5129]             # mm
real_rpy_deg = [3.14, 0, 0]                    # deg

# ===== 2. 单位转换 =====
xyz_m = np.array(real_xyz_mm) / 1000.0
rpy_rad = real_rpy_deg
target_pose = np.concatenate([xyz_m, rpy_rad])
print("🎯 Target pose (MuJoCo format):", target_pose)



# ===== 3. 实际初始角度（real_qpos）作为 q0 =====
real_qpos_deg = [0, 0, 0, 0, 0, 0]
real_qpos_rad = np.radians(real_qpos_deg)

# ===== 4. 加载模型并运行 IK =====
robot = get_robot("xarm6")

print("robot:", robot)
print("robot.q:", robot.q)
print("robot.qlim shape:", robot.qlim.shape)
print("robot.n:", robot.n)  # 关节数

qpos_ik, success = xarm6_lerobot_IK(real_qpos_rad, target_pose, robot=robot)


print("IK result :", qpos_ik)
print("Real qpos  :", real_qpos_deg)
print("Diff :", np.abs(qpos_ik- real_qpos_deg))

# ===== 5. 打印结果 =====
if success:
    print("✅ IK Success")
else:
    print("❌ IK Failed")
