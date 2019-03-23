#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from roscco.msg import ThrottleCommand
import struct

CAN_SPEED_ID = 0x52A
KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID = 0x386

class RosNode(object):
    def __init__(self):
        self.pub = rospy.Publisher('throttle_command', ThrottleCommand, queue_size=1)
        rospy.Subscriber('throttle', Float64, self.relay)
        rospy.init_node('relay', anonymous=True)
        rospy.spin()

    def relay(self, data):
        msg = ThrottleCommand()
        msg.throttle_position = data.data
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
