#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RosNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('raw_image', Image, queue_size=1)
        rospy.init_node('fake_camera_node', anonymous=True)
        cap = cv2.VideoCapture('/home/ranakhalil/Thunderhill2019/Soulless2019/movie.mp4')
        rate = rospy.Rate(5) # 6hz
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret is True:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                rospy.logerr("Failed to read image from camera")
                rospy.signal_shutdown("no more images")
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'rgb8'))
            rate.sleep()

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
