#!/usr/bin/env python
import roslib
import rospy
from roscco.msg import EnableDisable
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import serial

class RosNode(object):
    def __init__(self):
        rospy.init_node('master_node', anonymous=True)
        self.mode = rospy.get_param("~mode")
        self.enable_pub = rospy.Publisher('enable_disable', EnableDisable, queue_size=1)
        self.speed_pub = rospy.Publisher('/speed/setpoint', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('steering', Float64, queue_size=1)
    
    def loop(self):
        rospy.sleep(3)
        rospy.logerr(self.mode)
        if self.mode == 'command':
            msg = EnableDisable()
            # msg.header.stamp = 'now'
            msg.enable_control = True
            rospy.logerr(msg)
            self.enable_pub.publish(msg)
            msg = Float64()
            msg.data = 10
            self.speed_pub.publish(msg)
        if self.mode == 'test':
            rospy.logerr('In test mode')
            msg = EnableDisable()
            # msg.header.stamp = 'now'
            msg.enable_control = True
            self.enable_pub.publish(msg)
            msg = Float64()
            msg.data = 10
            self.speed_pub.publish(msg)
            msg = Float64()
            msg.data = 0
            self.steer_pub.publish(msg)
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RosNode()
        node.loop()
    except rospy.ROSInterruptException:
        pass
