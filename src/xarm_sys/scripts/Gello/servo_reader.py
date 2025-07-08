#!/usr/bin/env python3
import rospy
import serial
import time
import re
import numpy as np
from std_msgs.msg import Float64MultiArray

class ServoReaderNode:
    def __init__(self):
        rospy.init_node("servo_reader_node")
        self.pub = rospy.Publisher('/servo_angles', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(100)

        self.SERIAL_PORT = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
        rospy.loginfo("串口已打开")

        self.gripper_range = 0.48
        self.zero_angles = [0.0] * 7
        self._init_servos()

    def send_command(self, cmd):
        self.ser.write(cmd.encode('ascii'))
        time.sleep(0.008)
        return self.ser.read_all().decode('ascii', errors='ignore')

    def pwm_to_angle(self, response_str, pwm_min=500, pwm_max=2500, angle_range=270):
        match = re.search(r'P(\d{4})', response_str)
        if not match:
            return None
        pwm_val = int(match.group(1))
        pwm_span = pwm_max - pwm_min
        angle = (pwm_val - pwm_min) / pwm_span * angle_range
        return angle

    def _init_servos(self):
        self.send_command('#000PVER!')
        for i in range(7):
            self.send_command("#000PCSK!")
            self.send_command(f'#{i:03d}PULK!')
            response = self.send_command(f'#{i:03d}PRAD!')
            angle = self.pwm_to_angle(response.strip())
            self.zero_angles[i] = angle if angle is not None else 0.0
        rospy.loginfo("舵机初始角度校准完成")

    def run(self):
        angle_offset = [0.0] * 7
        while not rospy.is_shutdown():
            for i in range(7):
                response = self.send_command(f'#{i:03d}PRAD!')
                angle = self.pwm_to_angle(response.strip())
                if angle is not None:
                    angle_offset[i] = angle - self.zero_angles[i]
                else:
                    rospy.logwarn(f"舵机 {i} 回传异常: {response.strip()}")
            msg = Float64MultiArray(data=angle_offset)
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ServoReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
