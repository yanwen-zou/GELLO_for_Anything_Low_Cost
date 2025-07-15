from xarm.wrapper import XArmAPI  
import numpy as np

arm = XArmAPI('192.168.1.222')
arm.motion_enable(enable=True) 
arm.set_gripper_enable(enable=True) 
arm.set_mode(6)  
arm.set_state(0)  

init_qpos = np.array([4.3, 15.5, -9.4, 182.6, 100.4 ,0.3])
init_qpos = np.radians(init_qpos)

arm.set_servo_angle(angle=init_qpos,speed=0.5,is_qqradian=True)
arm.set_gripper_position(pos=730, wait=False)  # 设置夹持器位置为 50%，速度为 50%
      # 设置夹持力为 50%，可调q
      