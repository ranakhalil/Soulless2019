#!/usr/bin/env python
# license removed for brevity
import rospy
import torch
from torch import nn
import numpy as np
import cv2
import os
import onnx
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from Network.Network import SegNet
from torch.autograd import Variable
import caffe2.python.onnx.backend as backend

class RosNode(object):
    def __init__(self):
        self.seg_net = SegNet(15, dropProb = .2, ckpt_file = '/home/jendrik/logs/MapillaryOnly/mapillary_only.ckpt')
        self.seg_net.eval()
        self.seg_net.half()
        self.seg_net.cuda()
        self.bridge = CvBridge()
        self.has_onnx = os.path.exists('seg_net.onnx')
        # Load the ONNX model
        if self.has_onnx:
            model = onnx.load("seg_net.onnx")

            # Check that the IR is well formed
            onnx.checker.check_model(model)

            # Print a human readable representation of the graph
            onnx.helper.printable_graph(model.graph)
            self.onnx_model = backend.prepare(model, device="CUDA:0")
        self.pub = rospy.Publisher('segmented_image', Image, queue_size=1)
        rospy.Subscriber('raw_image', Image, self.detect)
        rospy.init_node('segmentation_node', anonymous=True)
        rospy.spin()


    def eval(self, x):
        self.seg_net.train(False)
        with torch.no_grad():
            _, res = nn.Softmax2d()(self.seg_net(x)).max(1)
        return res

    def detect(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        except CvBridgeError as e:
            print(e)
        # rospy.logerr(image.shape)
        dat = image / 255.
        dat = Variable(torch.from_numpy(dat).unsqueeze(0)).half().permute(0,3,1,2)
        dat = dat[:, :, 150:].cuda()
        if not self.has_onnx:

            # Providing input and output names sets the display names for values
            # within the model's graph. Setting these does not change the semantics
            # of the graph; it is only for readability.
            #
            # The inputs to the network consist of the flat list of inputs (i.e.
            # the values you would pass to the forward() method) followed by the
            # flat list of parameters. You can partially specify names, i.e. provide
            # a list here shorter than the number of inputs to the model, and we will
            # only set that subset of names, starting from the beginning.
            input_names = [ "image" ] + [ name for name, _ in self.seg_net.named_parameters()]
            output_names = [ "output"]
            torch.onnx.export(self.seg_net, dat, "seg_net.onnx", verbose=True, input_names=input_names, output_names=output_names)
            self.has_onnx = True
            model = onnx.load("seg_net.onnx")

            # Check that the IR is well formed
            onnx.checker.check_model(model)

            # Print a human readable representation of the graph
            onnx.helper.printable_graph(model.graph)
            self.onnx_model = backend.prepare(model, device="CUDA:0")

        # image = image[image.shape[0]//2:,:, :]
        # dat = image.reshape((-1, image.shape[0]//2, image.shape[1], image.shape[2]))
	res = self.onnx_model.run(dat)[0]
        tmp = (res == 14)
        res = np.dstack(((res==1)*255, tmp*255, (res==8)*255)).astype(np.uint8)
        self.pub.publish(self.bridge.cv2_to_imgmsg(res, 'rgb8'))
        # rospy.logerr("Published segmented image")

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
