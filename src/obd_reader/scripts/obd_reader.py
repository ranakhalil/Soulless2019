#!/usr/bin/env python
# license removed for brevity
import rospy
from roscco.msg import CanFrame
from std_msgs.msg import Float64
import struct

CAN_SPEED_ID = 0x52A
KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID = 0x386

class RosNode(object):
    def __init__(self):
        self.pub = rospy.Publisher('current_velocity', Float64, queue_size=1)
        rospy.init_node('obd_reader', anonymous=True)
        rospy.Subscriber('can_frame', CanFrame, self.extract_data)
        rospy.spin()

    def extract_data(self, data):
        frame = data.frame
        can_id = frame.can_id
        # rospy.logerr('id {}'.format(can_id))
        if can_id == KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID:
             if not frame.data:
                  return
             frame_data = [struct.unpack('B', x)[0] for x in frame.data]
             speed = 0
             for i in range(4):
                 raw = (frame_data[i*2+1] & 0x0F) << 8 | frame_data[i*2]
                 wheel_speed = int(raw / 3.2) / 10.0;
                 speed += wheel_speed
             speed /= 4
             msg = Float64()
             msg.data = speed
             self.pub.publish(msg)

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
