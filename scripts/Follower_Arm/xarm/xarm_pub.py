#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from xarm.wrapper import XArmAPI

class ArmInterfaceNode:
    def __init__(self):
        rospy.init_node('arm_inter')
        rospy.loginfo("Connecting to xArm...")

        # initialize xArm
        robot_ip = rospy.get_param("~robot_ip", "192.168.1.199")  #TODO: change IP to yours
        self.arm = XArmAPI(robot_ip)
        self.arm.motion_enable(True)
        self.arm.set_mode(6)  
        self.arm.set_state(0)  

        rospy.loginfo("xArm connected.")

        # Initialize ROS publishers
        self.state_pub = rospy.Publisher('/robot_state', Float64MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.publish_state)

    def publish_state(self, event):
        joint_angles = self.arm.get_servo_angle()
        gripper_pos = self.arm.get_gripper_position()

        if joint_angles and joint_angles[0] == 0:
            angles = joint_angles[1][:6]
        else:
            angles = [0.0] * 6

        if gripper_pos and gripper_pos[0] == 0:
            gpos = gripper_pos[1] 
        else:
            gpos = 0.0

        full_state = angles + [gpos]

        msg = Float64MultiArray()
        msg.data = full_state
        self.state_pub.publish(msg)
        rospy.loginfo(f"Published robot state: {msg.data}")


if __name__ == '__main__':
    try:
        ArmInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass