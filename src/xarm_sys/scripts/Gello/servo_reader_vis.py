#!/usr/bin/env python3
import rospy
import serial
import time
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from std_msgs.msg import Float64MultiArray

class ServoReaderNode:
    def __init__(self):
        rospy.init_node("servo_reader_node")
        self.pub = rospy.Publisher('/servo_angles', Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(100)
        self.SERIAL_PORT = '/dev/ttyUSB1'
        self.BAUDRATE = rospy.get_param("~baudrate", 115200)
        self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=0.1)
        rospy.loginfo("串口已打开")

        self.zero_angles = [0.0] * 7
        self._init_servos()

        # 可视化相关（仅 joint 0）
        self.window_seconds = 10
        self.update_hz = 30
        self.max_len = self.window_seconds * self.update_hz
        self.angle_history = deque(maxlen=self.max_len)
        self.time_history = deque(maxlen=self.max_len)
        self.start_time = time.time()

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label='Joint 0')
        self.ax.set_ylim(-180, 180)
        self.ax.set_title("Joint 0 Angle (last 10s)")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Angle (deg)")
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=1000//self.update_hz)

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

    def update_plot(self, frame):
        if len(self.time_history) < 2:
            return [self.line]
        t_vals = list(self.time_history)
        angles = list(self.angle_history)
        self.line.set_data(t_vals, angles)
        self.ax.set_xlim(max(0, t_vals[-1] - self.window_seconds), t_vals[-1])
        return [self.line]

    def run(self):
        angle_offset = [0.0] * 7
        target_angle_offset = [0.0] * 7
        num_interp = 5
        step_size = 1.5

        plt.ion()
        plt.show(block=False)

        while not rospy.is_shutdown():
            for i in range(7):
                response = self.send_command(f'#{i:03d}PRAD!')
                angle = self.pwm_to_angle(response.strip())
                if angle is not None:
                    new_angle = angle - self.zero_angles[i]
                    if abs(new_angle - target_angle_offset[i]) > step_size:
                        target_angle_offset[i] = new_angle
                else:
                    rospy.logwarn(f"舵机 {i} 回传异常: {response.strip()}")

            for step in range(num_interp):
                timestamp = time.time() - self.start_time
                delta = target_angle_offset[2] - angle_offset[2]
                angle_offset[2] += delta * 0.2
                self.time_history.append(timestamp)
                self.angle_history.append(angle_offset[2])

                print(f"[Joint 0] angle: {angle_offset[2]:.2f}")
                self.pub.publish(Float64MultiArray(data=angle_offset))
                self.rate.sleep()
                plt.pause(0.001)


if __name__ == '__main__':
    try:
        node = ServoReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
