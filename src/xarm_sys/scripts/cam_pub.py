#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class CameraNode:
    def __init__(self):
        rospy.init_node('cam_1')  # 示例用 cam-1
        #TODO: Put the camera initialization process here
        self.pub = rospy.Publisher('/cam-1', String, queue_size=10) 
        #TODO:Change String to Image when connecting to real cam. Here only for test.
        rospy.Timer(rospy.Duration(1), self.publish_image) #TODO: Frequency to be modified

    def publish_image(self, event):
        img_data = "simulated_image_data"
        self.pub.publish(img_data)
        rospy.loginfo("Published image data")

        #TODO: Get img through cam API. Following can be reference for processing

        '''
        color_frame = frames.get_color_frame()

        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        # 转换 OpenCV 图像为 ROS 1 消息
        ros_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')

        # 发布到 ROS 1
        self.publisher.publish(ros_image)
        rospy.loginfo(f'Published image from camera {self.serial}')
        self.pub.publish(img_data)
        rospy.loginfo("Published image data")
        '''

if __name__ == '__main__':
    CameraNode()
    rospy.spin()
