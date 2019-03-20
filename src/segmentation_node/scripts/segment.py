#!/usr/bin/env python
# license removed for brevity
import rospy
import torch
from torch import nn
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from Network.Network import SegNet
from torch.autograd import Variable

class RosNode(object):
    def __init__(self):
        self.seg_net = SegNet(15, dropProb = .2, ckpt_file = '/home/jendrik/logs/2019-03-17T19:10:48.816007SegNet/segNetClasses.ckpt')
        self.seg_net.half()
        self.seg_net.cuda()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('raw_image', Image, queue_size=1)
        self.pub = rospy.Publisher('segmented_image', Image, queue_size=1)
        rospy.init_node('segmentation_node', anonymous=True)
        self.camera_port = rospy.get_param("~camera_port")
        print("Camera Port", self.camera_port)
        cap = cv2.VideoCapture(self.camera_port)
        while not cap.isOpened():
            cap = cv2.VideoCapture(self.camera_port)
            cv2.waitKey(1000)
            rospy.logerr("Wait for the header")
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #frame = frame[::2,::2]
            if ret is False:
                rospy.logerr("Failed to read image from camera")
                continue
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'rgb8'))
            rospy.logerr("Published raw image")
            self.detect(frame)

    def eval(self, x):
        self.seg_net.train(False)
        with torch.no_grad():
            _, res = nn.Softmax2d()(self.seg_net(x)).max(1)
        return res

    def detect(self, image):
        # image = image[::2,::2,:]

        dat = image.reshape((-1, image.shape[0], image.shape[1], image.shape[2]))
        dat = dat / 255.
        dat = Variable(torch.from_numpy(dat)).half().permute(0,3,1,2)
        res = self.eval(dat.cuda()).data.cpu().numpy()[0]
        tmp = (res == 14)
        res = np.dstack(((res==1)*255, tmp*255, (res==8)*255)).astype(np.uint8)
        self.pub.publish(self.bridge.cv2_to_imgmsg(res, 'rgb8'))
        rospy.logerr("Published segmented image")

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
