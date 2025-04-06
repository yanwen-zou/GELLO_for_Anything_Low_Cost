#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray

class ArmInterfaceNode:
    def __init__(self):
        rospy.init_node('arm_inter')
        #TODO: Put arm initialization here
        self.cmd_sub = rospy.Subscriber('/robot_cmd', Float64MultiArray, self.cmd_callback)
        self.state_pub = rospy.Publisher('/robot_state', Float64MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(1), self.publish_state)

    def cmd_callback(self, msg):
        rospy.loginfo(f"Received command: {msg.data}")
        # 模拟调用 API 控制机械臂
        # self.robot.move(msg.data)

    def publish_state(self,event):
        # 模拟获取状态
        data_array = np.zeros(6) #TODO: Get actual robot state
        msg = Float64MultiArray()
        msg.data = data_array.tolist()
        self.state_pub.publish(msg)
        rospy.loginfo(f"Published robot state: {msg.data}")

if __name__ == '__main__':
    ArmInterfaceNode()
    rospy.spin()
