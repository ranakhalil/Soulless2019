#!/usr/bin/env python
# license removed for brevity
import rospy
from roscco.msg import CanFrame
from std_msgs.msg import Float64
import struct

# https://github.com/PolySync/oscc/blob/master/api/include/vehicles/kia_niro.h

CAN_SPEED_ID = 0x52A
KIA_SOUL_OBD_WHEEL_SPEED_CAN_ID = 0x386
KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID = 0x2B0
KIA_SOUL_OBD_STEERING_ANGLE_SCALAR = 0.1

class RosNode(object):
    def __init__(self):
        self.speed_pub = rospy.Publisher('current_velocity', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('current_steering_angle', Float64, queue_size=1)
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
             self.speed_pub.publish(msg)
        elif can_id == KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID:
             if not frame.data:
                  return
             frame_data = [struct.unpack('B', x)[0] for x in frame.data]
             speed = 0
             # rospy.logerr(frame_data)
             raw = (frame_data[1] << 8) | frame_data[0]
             # rospy.logerr(raw)
             steering_wheel_angle = -(float(raw) * KIA_SOUL_OBD_STEERING_ANGLE_SCALAR);
             if steering_wheel_angle < -3200:
                 steering_wheel_angle = 6553.5 + steering_wheel_angle
             # rospy.logerr(steering_wheel_angle)
             msg = Float64()
             msg.data = steering_wheel_angle
             self.steer_pub.publish(msg)


if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
