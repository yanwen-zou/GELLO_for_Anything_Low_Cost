from xarm.wrapper import XArmAPI  
import numpy as np
arm = XArmAPI('192.168.1.222')
arm.motion_enable(enable=True) 
arm.set_gripper_enable(enable=True) 
arm.set_mode(6)  
arm.set_state(0)  

init_qpos = np.array([14.1, -8, -24.7, 196.9, 62.3, -8.8])
init_qpos = np.radians(init_qpos)

arm.set_servo_angle(angle=init_qpos,speed=0.3,is_radian=True)
      # 设置夹持力为 50%，可调