#!/usr/bin/env python
import rospy
import numpy as np
import pickle
import os
import datetime
from std_msgs.msg import String, Float64MultiArray

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

class Recorder:
    def __init__(self):
        rospy.init_node('recorder')
        self.image_sub = rospy.Subscriber('/cam-1', String, self.image_callback)  # TODO: 改为Image类型
        self.state_sub = rospy.Subscriber('/robot_state', Float64MultiArray, self.state_callback)

        # self.bridge = CvBridge()  # 用于真实图像
        self.latest_image = None
        self.latest_state = None

        self.episode = []

        rospy.on_shutdown(self.save_episode)

        rospy.loginfo("Recorder initialized and listening...")

    def image_callback(self, msg):
        # 对于真实图像可以这样转换：
        # self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = msg.data  # 模拟图像（字符串）
        self.record()

    def state_callback(self, msg):
        self.latest_state = msg.data  # 是一个 list 或 ndarray
        self.record()

    def record(self):
        if self.latest_image is not None and self.latest_state is not None:
            ts = {
                "state": np.array(self.latest_state),
                "image": self.latest_image  # 如果是图像，通常是 np.array
            }
            self.episode.append(ts)
            rospy.loginfo(f"Recorded step {len(self.episode)}")

            # 清空，等待下一组数据
            self.latest_image = None
            self.latest_state = None

    def save_episode(self):
        if not self.episode:
            rospy.logwarn("No data to save.")
            return

        #TODO:Transfer to Lerobot Dataset Format    
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(os.path.expanduser("~"), f"lerobot_episode_{timestamp}.pkl")
        with open(save_path, 'wb') as f:
            pickle.dump(self.episode, f)
        rospy.loginfo(f"Episode saved to {save_path}")

if __name__ == '__main__':
    Recorder()
    rospy.spin()
