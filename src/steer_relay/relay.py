#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from roscco.msg import SteeringCommand
import struct

CAN_SPEED_ID = 0x52A
KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID = 0x386

class RosNode(object):
    def __init__(self):
        self.pub = rospy.Publisher('steering_command', SteeringCommand, queue_size=1)
        rospy.Subscriber('steer_torque', Float64, self.relay)
        rospy.init_node('steer_relay', anonymous=True)
        rospy.spin()

    def relay(self, data):
        msg = SteeringCommand()
        msg.steering_torque = max(min(data.data, .3), -.3)
        #rospy.logerr(msg)
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
