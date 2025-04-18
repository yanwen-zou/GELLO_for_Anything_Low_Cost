#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode:
    def __init__(self):
        rospy.init_node('cam_1')
        self.bridge = CvBridge()

        # 打开摄像头（默认设备0），GoPro一般也能识别为VideoCapture设备
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera.")
            raise RuntimeError("Cannot open camera.")

        self.pub = rospy.Publisher('/cam_1', Image, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.publish_image)  # 10Hz 可调

    def publish_image(self, event):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image.")
            return

        # OpenCV 默认是 BGR 格式
        ros_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(ros_img)
        rospy.loginfo("Published real image from camera")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
            rospy.loginfo("Camera released.")

if __name__ == '__main__':
    try:
        CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
