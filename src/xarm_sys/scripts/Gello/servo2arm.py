#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from xarm.wrapper import XArmAPI  
import numpy as np
import serial
import time
import re

class XArmTeleopNode:
    def __init__(self):
        rospy.init_node("xarm_teleop_node")
        self.pub = rospy.Publisher('/robot_action', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(20)

        self.SERIAL_PORT = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)

        self.init_qpos = np.radians([14.1, -8, -24.7, 196.9, 62.3, -8.8, 0.0])
        self.arm = XArmAPI('192.168.1.199')
        self._init_arm()
        self.arm_qpos = self.init_qpos

        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
        rospy.loginfo("串口已打开")
        self.gripper_range = 0.48
        rospy.Subscriber('/servo_angles', Float64MultiArray, self.servo_callback)

    def _init_arm(self):
        self.arm.motion_enable(enable=True) 
        self.arm.set_gripper_enable(enable=True) 
        self.arm.set_mode(6)  
        self.arm.set_state(0)  
        self.arm.set_servo_angle(angle=self.init_qpos, speed=1, is_radian=True)

    def servo_callback(self, msg):
        angle_offset = list(msg.data)
        self.arm_qpos = self.deg_angle_to_rad_xarm(angle_offset)


    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        return angle

    def angle_to_gripper(self, angle_deg, pos_min=50, pos_max=730):
        ratio = max(0, 1 - (angle_deg / self.gripper_range))
        position = pos_min + (pos_max - pos_min) * ratio
        return int(np.clip(position, pos_min, pos_max))

    def deg_angle_to_rad_xarm(self, angle_offset):
        angle_offset[2] = -angle_offset[2]
        angle_offset[6] = -angle_offset[6]
        rad_angle = np.radians(angle_offset) + self.init_qpos
        return rad_angle

    def run(self):
        while not rospy.is_shutdown():
            gripper_position = self.angle_to_gripper(self.arm_qpos[6])
            self.arm.set_servo_angle(angle=self.arm_qpos[:6], speed=0.5, is_radian=True)
            self.arm.set_gripper_position(gripper_position)
            
            arm_qpos_degree = [0.0] * 7
            arm_qpos_degree[:6] = np.degrees(self.arm_qpos[:6])
            arm_qpos_degree[6] = self.gripper_range - self.arm_qpos[6]

            msg = Float64MultiArray(data=list(arm_qpos_degree))
            self.pub.publish(msg)
            rospy.loginfo(f"Published robot action: {msg.data}")
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = XArmTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
