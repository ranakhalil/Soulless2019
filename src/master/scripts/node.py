#!/usr/bin/env python
import roslib
import rospy
from roscco.msg import EnableDisable
from sensor_msgs.msg import NavSatFix 
import serial

class RosNode(object):
    def __init__(self):
        rospy.init_node('master_node', anonymous=True)
        self.mode = rospy.get_param("~mode")
        self.enable_pub = rospy.Publisher('enable_disable', EnableDisable, queue_size=1)
    
    def loop(self):
        if self.mode == 'command':
            msg = EnableDisable()
            msg.header.stamp = 'now'
            msg.enable_control = True
            self.enable_pub.publish(msg)
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RosNode()
        node.loop()
    except rospy.ROSInterruptException:
        pass
