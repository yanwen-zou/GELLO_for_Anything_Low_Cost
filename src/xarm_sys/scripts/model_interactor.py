#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray

#This node should take the observation from hardwares, then call model,
# finally predict cmd sent to hardwares

class VLAInterNode:
    def __init__(self):
        rospy.init_node('vla_inter')
        self.image_sub = rospy.Subscriber('/cam-1', String, self.image_callback)  # 示例用 cam-1
        #TODO:Change String to Image when connecting to real cam. Here only for test.
        self.state_sub = rospy.Subscriber('/robot_state', Float64MultiArray, self.state_callback)
        self.cmd_pub = rospy.Publisher('/robot_cmd', Float64MultiArray, queue_size=10)

        self.latest_image = None
        self.latest_state = None

        self.policy = ACTpolicy()

    def image_callback(self, msg):
        self.latest_image = msg.data
        self.model_process()

    def state_callback(self, msg):
        self.latest_state = msg.data
        self.model_process()

    def model_process(self):
        if self.latest_image and self.latest_state:#TODO: Further latency matching

            # Take the observation from hardwares. Here is an example for test.
            cmd = self.policy(self.latest_image,self.latest_state)

            msg = Float64MultiArray()
            msg.data = cmd.tolist()
            self.cmd_pub.publish(msg)
            rospy.loginfo(f"Published command: {msg.data}")
            # 清空避免重复发送
            self.latest_image = None
            self.latest_state = None

class ACTpolicy: #Just an example for policy
    def __init__(self):
        pass
    def __call__(self,img,state):
        cmd = np.zeros(6)
        cmd += 1
        return cmd
if __name__ == '__main__':
    VLAInterNode()
    rospy.spin()
