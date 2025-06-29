import serial
import time
from xarm.wrapper import XArmAPI  
import numpy as np
import re

# 初始化机械臂
arm = XArmAPI('192.168.1.199')
arm.motion_enable(enable=True) 
arm.set_gripper_enable(enable=True) 
arm.set_mode(6)  
arm.set_state(0)  

init_qpos = np.array([14.1, -8, -24.7, 196.9, 62.3, -8.8, 0.0])
init_qpos = np.radians(init_qpos)

arm.set_servo_angle(angle=init_qpos, speed=1, is_radian=True)

# 设置串口参数
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

def send_command(ser, cmd):
    ser.write(cmd.encode('ascii'))
    time.sleep(0.008)
    response = ser.read_all()
    return response.decode('ascii', errors='ignore')

def pwm_to_angle(response_str, pwm_min=500, pwm_max=2500, angle_range=270):
    match = re.search(r'P(\d{4})', response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    pwm_span = pwm_max - pwm_min
    angle = (pwm_val - pwm_min) / pwm_span * angle_range
    return angle

def angle_to_gripper(angle_deg, angle_range=0.48, pos_min=50, pos_max=730):
    """
    将舵机角度（度）映射为 gripper position。

    参数:
    - angle_deg: 舵机角度，单位度
    - angle_range: 舵机最大角度（默认270°）
    - pos_min: gripper闭合位置（默认50）
    - pos_max: gripper张开位置（默认730）

    返回:
    - gripper position（整型）
    """
    ratio = max(0, 1-(angle_deg / angle_range))
    position = pos_min + (pos_max - pos_min) * ratio
    return int(np.clip(position, pos_min, pos_max))

def deg_angle_to_rad_xarm(angle_offset):

    angle_offset[2] = -angle_offset[2]
    #angle_offset[4] = -angle_offset[4]
    angle_offset[6] = -angle_offset[6]

    rad_angle = np.radians(angle_offset)

    rad_angle = rad_angle + init_qpos

    return rad_angle


def main():
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1) as ser:
        print("串口已打开")

        response = send_command(ser, '#000PVER!')
        print(f"版本号回复: {response.strip()}")
        zero_angles = [0.0] * 7
        # 依次释放所有舵机扭力
        for i in range(7):
            send_command(ser, "#000PCSK!")
            cmd = f'#{i:03d}PULK!'
            response = send_command(ser, cmd)
            print(f"舵机 {i} 释放扭力: {response.strip()}")
            cmd = f'#{i:03d}PRAD!'
            response = send_command(ser, cmd)
            angle = pwm_to_angle(response.strip())
            zero_angles[i] = angle
        
        arm_qpos = [0.0] * 7
        angle_offset = [0.0] * 7
        while True:
            for i in range(7):  # ID 从 0 到 6
                
                cmd = f'#{i:03d}PRAD!'
                response = send_command(ser, cmd)
                angle = pwm_to_angle(response.strip())
                if angle is not None:
                    angle_offset[i] = angle - zero_angles[i]
                else:
                    print(f"舵机 {i} 回传: {response.strip()}")
            arm_qpos = deg_angle_to_rad_xarm(angle_offset)
            print(f"arm_pos:{arm_qpos}")
            gripper_position = angle_to_gripper(arm_qpos[6])
            arm.set_servo_angle(angle=arm_qpos[:6], speed=0.5, is_radian=True)
            arm.set_gripper_position(gripper_position)
            

if __name__ == "__main__":
    main()
