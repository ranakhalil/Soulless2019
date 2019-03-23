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
import time

class RosNode(object):
    def __init__(self):
        self.seg_net = SegNet(15, dropProb = .2, ckpt_file = '/home/jendrik/logs/MapillaryOnly/mapillary_only.ckpt')
        self.seg_net.half()
        self.seg_net.cuda()
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('segmented_image', Image, queue_size=1)
        rospy.Subscriber('raw_image', Image, self.detect, queue_size=2, buff_size=2**24)
        rospy.init_node('segmentation_node', anonymous=True)
        rospy.spin()


    def eval(self, x):
        self.seg_net.train(False)
        with torch.no_grad():
            _, res = nn.Softmax2d()(self.seg_net(x)).max(1)
        return res

    def detect(self, data):
        start = time.time()
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        except CvBridgeError as e:
            print(e)
        dat = image / 255.
        dat = dat[150:]
        dat = Variable(torch.from_numpy(dat).unsqueeze(0)).half().permute(0,3,1,2)
        res = self.eval(dat.cuda()).data.cpu().numpy()[0]
        tmp = (res == 14)
        res = np.dstack(((res==1)*255, tmp*255, (res==8)*255)).astype(np.uint8)
        self.pub.publish(self.bridge.cv2_to_imgmsg(res, 'rgb8'))
        rospy.logerr("Segmenting image took {}".format(time.time()-start))

if __name__ == '__main__':
    try:
        node = RosNode()
    except rospy.ROSInterruptException:
        pass
