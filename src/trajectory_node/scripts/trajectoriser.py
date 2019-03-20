#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose, PoseArray, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from torch.autograd import Variable
from torch.nn import AvgPool2d as AvgPool2d
import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

MEDIAN_THRESHOLD = 50


class RosNode(object):
    def __init__(self):
        self.polyArr = []
        rospy.init_node('segmenter', anonymous=True)
        self.simple_controller_mode = rospy.get_param("~simple_steer")
        if self.simple_controller_mode:
            self.polyDeg = rospy.get_param("~poly_deg")

        self.bridge = CvBridge()

        self.pub = rospy.Publisher('/segnet/trajectory', PoseArray, queue_size=1)
        self.paramPub = rospy.Publisher('/segnet/polyparam', Float32MultiArray, queue_size=1)
        if self.simple_controller_mode == True:
            self.twistPub = rospy.Publisher('/steering', Twist, queue_size=1)
        self.imgPub = rospy.Publisher('filterImg', Image, queue_size=1)
        rospy.Subscriber('segmentedImage', Image, self.callback)
        rospy.spin()

    def runsOfZerosArray(self, bits, roadBits):
        # make sure all runs of ones are well-bounded
        bounded = np.hstack(([0], bits > 0, [0]))
        # get 1 at run starts and -1 at run ends
        difs = np.diff(bounded)
        runStarts = np.where(difs < 0)[0]
        runEnds = np.where(difs > 0)[0]
        runStarts = np.insert(runStarts, 0, 0)
        runEnds = np.append(runEnds, len(bits)-1)
        if len(runStarts) == 0:
            return [], 0, 0
        #print(runStarts, runEnds)
        lengths = runEnds - runStarts
        pixelLengths = []
        for start, end in zip(runStarts, runEnds):
            pixelLengths.append(np.sum(roadBits[start:end] > 0))
        pixelLengths = np.array(pixelLengths)
        idx = np.argmax(pixelLengths)
        return runStarts, idx, lengths[idx], pixelLengths[idx]

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        except CvBridgeError as e:
            print(e)
        filteredAnnotation = self.filterByCones(image)
        #filteredAnnotation = self.filterRoad(filteredAnnotation)
        self.imgPub.publish(self.bridge.cv2_to_imgmsg(filteredAnnotation, 'rgb8'))
        xs, ys = self.medianTrajectory(filteredAnnotation)
        poseList = []
        for x, y in zip(xs,ys):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            poseList.append(pose)
        arr = PoseArray()
        arr.poses = poseList
        self.pub.publish(arr)
        msg = Float32MultiArray()
        msg.data = self.polyArr
        self.paramPub.publish(msg)
        if self.simple_controller_mode:
            msg = Twist()
            msg.linear.x = .0
            msg.angular.x = -np.arctan(self.polyArr[self.polyDeg-1])*3*180/np.pi
            self.twistPub.publish(msg)

    def filterRoad(self, annotation):
        upperLim = annotation.shape[0]//2 - 30
        lowerLim = annotation.shape[0] - MEDIAN_THRESHOLD
        avgRegion = 9
        pad = (avgRegion-1) // 2

        anns = annotation.astype(np.float32)
        anns = Variable(torch.from_numpy(anns[upperLim:lowerLim,:,1].reshape((1,1, lowerLim-upperLim, anns.shape[1])))).cuda()
        meanMap = AvgPool2d(kernel_size=avgRegion, stride=1, padding=pad)(anns).cpu().data.numpy().reshape((lowerLim-upperLim, annotation.shape[1]))
        leftIndices = None
        rightIndices = None
        lastStart = -1
        lastEnd = -1
        startTol = 20
        endTol = 20

        for i in range(meanMap.shape[0])[::-1]:
            starts, idx, length, pixelLengths = self.runsOfZerosArray(meanMap[i], annotation[upperLim+i,:,0])

            if lastStart != -1:
                if len(starts) == 0 or pixelLengths < 50: continue
                if starts[idx] == 0:
                    if np.all(leftIndices[:,1] == 0):
                        leftIndices = np.append(leftIndices, [[i + upperLim, starts[idx]]], axis=0)
                        lastStart = starts[idx]
                        startTol = 200
                elif np.abs(starts[idx] - lastStart) < startTol:
                    leftIndices = np.append(leftIndices, [[i + upperLim, starts[idx]]], axis=0)
                    lastStart = starts[idx]
                    startTol = 30
                else: startTol += 10
                if starts[idx] + length == annotation.shape[1] -1:
                    if np.all(rightIndices[:,1] == annotation.shape[1] -1) :
                        rightIndices = np.append(rightIndices, [[i + upperLim, starts[idx]+length]], axis=0)
                        lastEnd = starts[idx] + length
                        endTol = 200
                elif np.abs(starts[idx] + length - lastEnd) < endTol:
                    rightIndices = np.append(rightIndices, [[i + upperLim, starts[idx] + length]], axis=0)
                    lastEnd = starts[idx] + length
                    endTol = 30
                else: endTol += 5
            else:
                leftIndices = np.array([[i + upperLim, starts[idx]]])
                rightIndices = np.array([[i + upperLim, starts[idx] + length]])
                lastStart = starts[idx]
                lastEnd = starts[idx] + length
        leftIndices = np.array(leftIndices)
        rightIndices = np.array(rightIndices)
        if(len(leftIndices) == 0 or len(rightIndices) == 0): return annotation
        leftIndicesY = np.arange(np.min(leftIndices[:,0]), np.max(leftIndices[:,0]), 1)
        leftIndicesX = np.interp(leftIndicesY,
                            leftIndices[:,0][::-1], leftIndices[:,1][::-1])

        rightIndicesY = np.arange(np.min(rightIndices[:,0]), np.max(rightIndices[:,0]), 1)
        rightIndicesX = np.interp(rightIndicesY,
                            rightIndices[:,0][::-1], rightIndices[:,1][::-1])
        for x, y in zip(rightIndicesX, rightIndicesY):
            annotation[int(y), int(x):, 0] = 0
            annotation[int(y), :int(x), 2] = 255
        for x, y in zip(leftIndicesX, leftIndicesY):
            annotation[int(y), :int(x), 0] = 0
            annotation[int(y), :int(x), 2] = 0
        return annotation


    def filterByCones(self, annotation):
        upperLim = annotation.shape[0] // 4
        lowerLim = annotation.shape[0] *6 //10
        sideIndent = 150
        leftLim = sideIndent
        rightLim = annotation.shape[1] - sideIndent

        avgRegion = (21,5)
        pad = (10,2)

        anns = annotation.astype(np.float32)
        anns = anns[upperLim:lowerLim:,leftLim:rightLim,2].reshape((1,1, lowerLim-upperLim, rightLim-leftLim))[:,:,::-1].copy()
        anns = Variable(torch.from_numpy(anns)).cuda()
        meanMap = AvgPool2d(kernel_size=avgRegion, stride=1, padding=pad)(anns)
        minIndex = torch.max(torch.max(meanMap, 3)[0],2)
        if minIndex[0].cpu().data.numpy() < 254:
            return annotation
        minIndex = int(minIndex[1].cpu().data.numpy())
        annotation[:lowerLim-minIndex,:,0] = 0
        return annotation

    def medianTrajectory(self, annotation):
        upperLim = annotation.shape[0] // 2 -80#- MEDIAN_THRESHOLD
        #upperLim = annotation.shape[0] - 2*MEDIAN_THRESHOLD
        lowerLim = annotation.shape[0]
        roadIndices = []
        for i in np.arange(upperLim, lowerLim, 1):
            indices = np.where(annotation[i, :, 0] > 0)
            indices = indices[0]
            if len(indices) > 20:
                roadIndices.append([lowerLim - i + (annotation.shape[0] - lowerLim), int(np.mean(indices))])

        roadIndices = np.array(roadIndices)

        filteredRoadIndices = []#[0, annotation.shape[1]//2]]
        filterLen = 3

        for i in range(len(roadIndices)-filterLen*2):
            val = 0
            for j in np.arange(-1*filterLen,filterLen,1):
                val += roadIndices[i-j,1]
            val //= filterLen * 2 + 1
            filteredRoadIndices.append([roadIndices[i,0], val])
        filteredRoadIndices = np.array(filteredRoadIndices)
        weights = np.ones(len(filteredRoadIndices))
        #if self.simple_controller_mode: weights[0] = 1e9
        #weights[0] = 1e9
        # Giving the MPC quadratic values
        poly_order = 3
        if self.simple_controller_mode == True:
            poly_order = self.polyDeg
        z = np.polyfit(filteredRoadIndices[:,0], filteredRoadIndices[:,1], poly_order, w=weights)

        if len(self.polyArr) == 0:
            self.polyArr = z
        else:
            self.polyArr = .7 * self.polyArr + .3 * z
        p = np.poly1d(self.polyArr)

        xs = []
        ys = []

        for y in np.arange(0, lowerLim-upperLim, 1):
            xs.append(p(y))
            ys.append(y)
        return xs, ys

if __name__ == '__main__':
    try:
        RosNode()
    except rospy.ROSInterruptException:
        pass
