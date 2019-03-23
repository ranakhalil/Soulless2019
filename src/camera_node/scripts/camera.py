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
        rospy.init_node('camera_node', anonymous=True)
        self.camera_port = rospy.get_param("~camera_port")
        self.cap = cv2.VideoCapture(self.camera_port)
        self.rate = rospy.Rate(20)
        while not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_port)
            cv2.waitKey(1000)
            rospy.logerr("Wait for the header")
    
    def loop(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #frame = frame[::2,::2]
            if ret is False:
                rospy.logerr("Failed to read image from camera")
                continue
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'rgb8'))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = RosNode()
        node.loop()
    except rospy.ROSInterruptException:
        pass
