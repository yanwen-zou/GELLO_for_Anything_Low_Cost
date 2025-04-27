#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DualCamViewer:
    def __init__(self):
        rospy.init_node('cam_sub_test')
        self.bridge = CvBridge()

        rospy.Subscriber('/cam_1', Image, self.cam1_callback)
        rospy.Subscriber('/cam_2', Image, self.cam2_callback)

        self.frame1 = None
        self.frame2 = None

        self.run()

    def cam1_callback(self, msg):
        try:
            self.frame1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"cam1 callback error: {e}")

    def cam2_callback(self, msg):
        try:
            self.frame2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"cam2 callback error: {e}")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.frame1 is not None:
                cv2.imshow("Camera 1 (/cam_1)", self.frame1)
            if self.frame2 is not None:
                cv2.imshow("Camera 2 (/cam_2)", self.frame2)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        DualCamViewer()
    except rospy.ROSInterruptException:
        pass
