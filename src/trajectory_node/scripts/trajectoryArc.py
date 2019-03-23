#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import math
from math import ceil
from cv_bridge import CvBridge, CvBridgeError

TRAJECTOR_PIXELS = [10711, 12315, 13894, 9520, 13894, 12315, 10711]
STEERING_RATIOS = [-1.0, -0.6, -0.3, 0, 0.3, 0.6, 1.0]
R = [100, 150, 200]

class RosNode(object):
    def __init__(self):
        self.steering_theta = 0.0
        self.horizon = 0.0
        self.height = 0.0
        self.width = 0.0
        self._params_initialized = False
        self.visualize = False
        self.alpha = 0.2
        rospy.init_node('trajectory_arc_node', anonymous= True )
        self.time_since_last_update = rospy.get_rostime().secs
        self.bridge = CvBridge()
        self.steeringTwistPub = rospy.Publisher('/steering', Twist, queue_size=1)
        self.imageTrajectoryPub = rospy.Publisher('/trajectory_image', Image, queue_size=1)
        rospy.Subscriber('/segmented_image', Image, self.callback)
        rospy.spin()

    def is_red_pixel(self, image, x, y):
        if image[y, x, 2] == 255 and image[y, x, 0] == 0 and image[y, x, 1] == 0:
            return 1
        return 0

    def softmax(self, x):
        """Compute softmax values for each sets of scores in x."""
        return np.exp(x) / np.sum(np.exp(x), axis=0)

    ## Center Trajectories
    def center_trajectories(self, image, r, visualize=True):
        red_pixel_count = 0
        for y in range(self.height - 50, self.horizon , -1):
            xL = int((ceil( self.width / 2. ) -r))
            xR = int((ceil( self.width / 2. ) +r))
            for x in range(xL, xR):
                red_pixel_count += self.is_red_pixel(image, x, y)
                if visualize and int(image[y, x, 2] == 255 and image[y, x, 0] == 0 and image[y, x, 1] == 0):
                    image[y,x] = (255, 255, 255)
                elif int(image[y, x, 2] == 0 and image[y, x, 0] == 0 and image[y, x, 1] == 255):
                    return red_pixel_count
        return red_pixel_count

    ## Right Trajectories
    def right_trajectories(self, image, R, r, LTolerance, visualize=True):
        red_pixel_count = 0
        for y in range(self.height - 50, self.horizon , -1):
            xL = self.width
            xR = self.width
            if ( R + r ) * ( R + r )-( y - self.height ) * ( y - self.height) >= 0 :
                xL = int((ceil( self.width / 2. )+(R-r))-math.sqrt(( R + r ) * ( R + r ) - ( y-self.height )*(y - self.height)))
            if (R-r) * (R-r)-( y - self.height ) * ( y - self.height ) >= 0:
                xR = int((ceil( self.width /2. )+(R+r))-math.sqrt((R-r)*(R-r)-( y-self.height )*( y-self.height )))
            xL = max( min( xL, self.width ), 0 )
            xR = max( min( xR, self.width ), 0 )
            x_count = 0
            for x in range(xL, xR):
                red_pixel_count += self.is_red_pixel(image, x, y)
                if visualize and int(image[y, x, 2] == 255 and image[y, x, 0] == 0 and image[y, x, 1] == 0):
                    image[y,x] = (255, 255, 255)
                elif int(image[y, x, 2] == 0 and image[y, x, 0] == 0 and image[y, x, 1] == 255):
                    if (x_count < LTolerance):
                        return red_pixel_count
                    else:
                        break
                x_count += 1
        return red_pixel_count

    ## Left Trajectories
    def left_trajectories(self, image, R, r, LTolerance, visualize=True):
        red_pixel_count = 0
        for y in range(self.height - 50, self.horizon , -1):
            xL = 0
            xR = 0
            if (R-r)*(R-r)-( y-self.height )*( y-self.height ) >= 0 :
                xL = int((ceil( self.width/2. )-(R+r)) + math.sqrt((R-r) * (R-r)-( y-self.height )*( y-self.height )))
            if (R+r) * (R+r) - (y-self.height) * (y-self.height) >= 0:
                xR = int((ceil( self.width/2. )-(R-r)) + math.sqrt((R+r)*(R+r)-( y-self.height )*( y-self.height )))
            xL = max(min( xL, self.width ),0)
            xR = max(min( xR, self.width ),0)
            x_count = 0
            for x in range(xR - 1, xL-1, -1):
                red_pixel_count += self.is_red_pixel(image, x, y)
                if visualize and int(image[y, x, 2] == 255 and image[y, x, 0] == 0 and image[y, x, 1] == 0):
                    image[y,x] = (255, 255, 255)
                elif int(image[y, x, 2] == 0 and image[y, x, 0] == 0 and image[y, x, 1] == 255):
                    if (x_count < LTolerance):
                        return red_pixel_count
                    else:
                        break
                x_count += 1
        return red_pixel_count


    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        shape = image.shape
        if not self._params_initialized:
            self.height = shape[0]
            self.width = shape[1] - 1
            self.horizon = int(self.height * 0.4)
            self._params_initialized = True
            rospy.loginfo("Image Params has been initialized")

        results = []
        results.append(self.left_trajectories(image, R[0], 10, 10, False))
        results.append(self.left_trajectories(image, R[1], 10, 10, False))
        results.append(self.left_trajectories(image, R[2], 10, 10, False))

        results.append(self.center_trajectories(image, 20, False))

        results.append(self.right_trajectories(image, R[2], 10, 10, False))
        results.append(self.right_trajectories(image, R[1], 10, 10, False))
        results.append(self.right_trajectories(image, R[0], 10, 10, False))
        
        results = np.array( results, dtype = float ) / np.array( TRAJECTOR_PIXELS, dtype = float )
        results = self.softmax(results)

        results = np.dot(results, STEERING_RATIOS)

        self.steering_theta = self.alpha * self.steering_theta + (1 - self.alpha) * results

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.x = self.steering_theta
        self.steeringTwistPub.publish(msg)
        
        if self.visualize:
            image_cpy = image.copy()
            self.left_trajectories(image_cpy, R[0], 10, 10)
            self.left_trajectories(image_cpy, R[1], 10, 10)
            self.left_trajectories(image_cpy, R[2], 10, 10)
            self.center_trajectories(image_cpy, 20)
            self.right_trajectories(image_cpy, R[2], 10, 10)
            self.right_trajectories(image_cpy, R[1], 10, 10)
            self.right_trajectories(image_cpy, R[0], 10, 10)
            self.imageTrajectoryPub.publish(self.bridge.cv2_to_imgmsg(image_cpy, "bgr8"))

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
