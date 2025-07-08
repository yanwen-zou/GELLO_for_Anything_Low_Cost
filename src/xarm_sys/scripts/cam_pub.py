#!/usr/bin/env python3
import rospy
import cv2
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class CameraNode:
    def __init__(self):
        rospy.init_node('cam_1')
        self.bridge = CvBridge()

        # 配置 RealSense 相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 配置颜色流，分辨率和帧率
        self.pipeline.start(config)

        self.pub = rospy.Publisher('/cam_1', Image, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.publish_image)  # 10Hz 可调

    def publish_image(self, event):
        # 获取 RealSense 的图像数据
        frames = self.pipeline.wait_for_frames()  # 获取帧
        color_frame = frames.get_color_frame()  # 获取颜色帧
        if not color_frame:
            rospy.logwarn("Failed to capture image.")
            return

        frame = np.asanyarray(color_frame.get_data())  # 转换为 NumPy 数组

        # OpenCV 默认是 BGR 格式
        ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(ros_img)
        rospy.loginfo("Published real image from camera")

    def __del__(self):
        self.pipeline.stop()  # 停止流
        rospy.loginfo("Camera released.")

if __name__ == '__main__':
    try:
        CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
