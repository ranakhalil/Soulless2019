#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix 
import serial

class RosNode(object):
    def __init__(self):
        rospy.init_node('gps_node', anonymous=True)
        self.serial_port = rospy.get_param("~serial_port")
        self.gps_pub = rospy.Publisher('gps', NavSatFix, queue_size=1)
        self.rate = rospy.Rate(10)
        self.ser = serial.Serial(self.serial_port, 115200)
    
    def loop(self):
        while not rospy.is_shutdown():
            data = self.ser.readline()
            if data and '*' not in data:
                data = data.split()
                msg = NavSatFix()
                msg.latitude = float(data[1])
                msg.longitude = float(data[2])
                msg.altitude = float(data[3])
                self.gps_pub.publish(msg)
            else:
                rospy.logerr("Got {}".format(data))
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = RosNode()
        node.loop()
    except rospy.ROSInterruptException:
        pass
