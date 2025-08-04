#!/usr/bin/env python3
import rospy
import numpy as np
import pickle
import os
import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from threading import Lock

class Recorder:
    def __init__(self):
        rospy.init_node('recorder')
        self.bridge = CvBridge()
        self.lock = Lock()

        self.latest_image_cam_1 = None  # 用于存储第一个相机图像
        self.latest_image_cam_2 = None  # 用于存储第二个相机图像
        self.latest_state = None
        self.latest_action = None
        self.episode = []

        # 订阅相机数据
        rospy.Subscriber('/cam_1', Image, self.image_callback_cam_1)
        rospy.Subscriber('/cam_2', Image, self.image_callback_cam_2)
        rospy.Subscriber('/robot_state', Float64MultiArray, self.state_callback)
        rospy.Subscriber('/robot_action', Float64MultiArray, self.action_callback)
        rospy.on_shutdown(self.save_episode)
        rospy.loginfo("Recorder initialized and listening...")

    def image_callback_cam_1(self, msg):
        rospy.loginfo("Image callback triggered for cam_1")  # 这行输出帮助确认回调是否触发
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image from cam_1: {e}")
            return
        with self.lock:
            self.latest_image_cam_1 = cv_image
            self.try_record()
            rospy.loginfo("Image received from camera 1")


    def image_callback_cam_2(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image from cam_2: {e}")
            return

        with self.lock:
            self.latest_image_cam_2 = cv_image
            self.try_record()
            rospy.loginfo("Image received from camera 2")

    def state_callback(self, msg):
        with self.lock:
            self.latest_state = msg.data
            self.try_record()
            rospy.loginfo("State received")

    def action_callback(self, msg):
        with self.lock:
            self.latest_action = msg.data
            self.try_record()
            rospy.loginfo("Action received")

    def try_record(self):
        if self.latest_image_cam_1 is not None and self.latest_image_cam_2 is not None and self.latest_state is not None and self.latest_action is not None:
            ts = {
                "image_cam_1": self.latest_image_cam_1.copy(),
                "image_cam_2": self.latest_image_cam_2.copy(),
                "state": np.array(self.latest_state),
                "action": np.array(self.latest_action)
            }
            self.episode.append(ts)
            rospy.loginfo(f"Recorded step {len(self.episode)}")
            
            # Reset the images and state for next step
            self.latest_image_cam_1 = None
            self.latest_image_cam_2 = None
            self.latest_state = None
            self.latest_action = None
        else:
            missing = []
            if self.latest_image_cam_1 is None:
                missing.append("image_cam_1")
            if self.latest_image_cam_2 is None:
                missing.append("image_cam_2")
            if self.latest_action is None:
                missing.append("action")
            if self.latest_state is None:
                missing.append("state")
            
            rospy.loginfo(f"Missing: {', '.join(missing)}")

    def save_episode(self):
        if not self.episode:
            rospy.logwarn("No data recorded, skipping save.")
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"lerobot_episode_{timestamp}.pkl"
        save_dir = "/home/zhaobo/code/teleop2/save_data/bread_mixed_oreo"
        os.makedirs(save_dir, exist_ok=True)  # 自动创建目录
        save_path = os.path.join(save_dir, filename)

        try:
            with open(save_path, 'wb') as f:
                pickle.dump(self.episode, f)
            rospy.loginfo(f"Episode saved to {save_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save episode: {e}")


if __name__ == '__main__':
    try:
        Recorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
