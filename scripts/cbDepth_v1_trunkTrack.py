#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/p_1_45_center_try/"
    # print(sys.path)
# import message_filters

'''math tool'''
import csv
import numpy as np
import time
import statistics
np.set_printoptions(threshold=sys.maxsize)

'''image tool'''
import cv2
import myUtils
from cv_bridge import CvBridge, CvBridgeError

'''plot tool'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.ticker as ticker

'''ball tracker'''
from collections import deque
pts = deque(maxlen=5)


def msg2CV(msg):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return image
    except CvBridgeError as e:
        print(e)
        return

flag = False
param_model = np.zeros((480,640))

def fnGroundSeg(npColor, npDepth, file_index):
    global flag, param_model, pts
    '''filter out 3m'''
    height,width = npDepth.shape
    npDepthF = cv2.convertScaleAbs(npDepth, alpha=0.085) # 6m
    # npDepthF_color = cv2.applyColorMap(npDepthF, cv2.COLORMAP_JET)
    # cv2.imshow("filtered", npDepthF_color)
    # cv2.waitKey(500)

    npDepth_binary = np.copy(npDepth)
    npDepth_binary = npDepth_binary.astype('float32')

    ret, npDepth_binary1 = cv2.threshold(npDepth_binary, 3000, 255, cv2.THRESH_BINARY_INV)
    ret, npDepth_binary2 = cv2.threshold(npDepth_binary, 500, 255, cv2.THRESH_BINARY)
    npDepth_binary1 = npDepth_binary1.astype('uint8')
    npDepth_binary2 = npDepth_binary2.astype('uint8')

    npDepth_binary = cv2.bitwise_and(npDepth_binary1, npDepth_binary2)
    npDepth_binary = npDepth_binary.astype('uint8')

    '''world coordinate'''
    cx_d = 328 #424
    cy_d = 241 #241
    fx_d = 617 #424
    fy_d = 617 #424
    npPointX = np.asarray(range(640))-cx_d
    npPointX = np.diag(npPointX)
    npPointX = npDepth.dot(npPointX)/ fx_d * (-1)

    npPointY = np.asarray(range(480))-cy_d
    npPointY = np.diag(npPointY)

    theta = 0/180*np.pi
    npPointY = npPointY.dot(npDepth)/ fy_d * (-1) 
    npPointY = npPointY*np.cos(theta) + npDepth * np.sin(theta) + 360
    npPointY = npPointY.astype('float16')

    '''filter out 5mm'''
    npHeight = np.copy(npPointY)
    npHeight = npHeight.astype('float32')

    ret, npHeight_binary = cv2.threshold(npHeight, 5, 255, cv2.THRESH_BINARY)
    npHeight_binary = npHeight_binary.astype('uint8')

    if len(param_model) == 5:
        flag = True
        print("BD: initialize param ground!")
        param_model = param_model/5
        moving_avg = np.zeros_like(param_model)
        moving_avg[param_model<=0.5]=int(0)
        moving_avg[param_model>0.5]=int(255)
        moving_avg = moving_avg.astype('uint8')
        print(len(param_model))

    elif len(param_model) < 5:
        print("BD: not yet initial!")
        moving_avg = npHeight_binary
        param_model = param_model + npHeight_binary
        print(len(param_model))
    else: 
        print('BD: '+str(len(param_model))+"initialized")
        alpha=0.2
        param_model = (1-alpha)*param_model+alpha*npHeight_binary
        param_model = param_model.astype('uint8')
        moving_avg = np.zeros_like(param_model)
        moving_avg[param_model<=130]=int(0)
        moving_avg[param_model>130]=int(255)

    npTreeMask = cv2.bitwise_and(npDepth_binary, moving_avg)
    mask = np.zeros_like(npTreeMask)
    region_of_interest_vertices = [(0,height/2),(0, height),(width, height),(width,height/2)]
    cv2.fillPoly(mask, np.array([region_of_interest_vertices], np.int32), 255)
    npTreeMask = cv2.bitwise_and(npTreeMask, mask)
    
    npTreeMask_c = cv2.cvtColor(npTreeMask, cv2.COLOR_GRAY2BGR)
    npTreeMask_c = cv2.addWeighted(npTreeMask_c, 0.5, npColor, (0.6), 0.0)

    contours, hierarchy = cv2.findContours(npTreeMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
    npDepthF_copy = np.copy(npDepthF)
    npDepthF_copy = cv2.cvtColor(npDepthF_copy, cv2.COLOR_GRAY2BGR)

    '''extract the only obj'''
    allObj_plot = np.zeros(npDepth.shape, dtype='uint8')
    moments = []
    index = []
    objSizeBound = [8000, 35000]
    i = 0
    for ct in contours:
        obj_plot = np.zeros(npDepth.shape, dtype='uint8')
        cv2.fillPoly(obj_plot, [ct], 255)
        m = cv2.moments(ct)
        if m["m00"] > objSizeBound[0] and m["m00"] < objSizeBound[1]:
            cv2.fillPoly(allObj_plot, [ct], 255)
            moments.append(m)
            index.append(i)
        i = i+1
            
    if len(index) != 0:
        (x, y, w, h) = cv2.boundingRect(contours[index[0]])
        if w/h < 3:
            cv2.rectangle(npTreeMask_c, (x, y), (x + w, y + h), (0, 255, 0), 2)
            m = moments[0]
            cX = int(m['m10'] / m['m00'])
            cY = int(m['m01'] / m['m00'])
            cv2.circle(npDepthF_copy, (cX, cY), 10, (1, 227, 254), -1)
            cv2.circle(npTreeMask_c, (cX, cY), 10, (1, 227, 254), -1)

            # update the points queue
            pts.appendleft((cX,cY))
            if len(pts) > 2: # at least receive 2 pts
                print("receive!")
                for i in range(1, len(pts)):
                    thickness = int(np.sqrt(5 / float(i + 1)) * 2.5)
                    cv2.line(npTreeMask_c, pts[i - 1], pts[i], (0, 0, 255), thickness)

            '''average depth'''
            accu_t_depth = 0
            tree_depth = np.array([])
            accu_t_amount = 0
            for i in range(y, y+h):
                for j in range(x, x+w):
                    if npTreeMask[i][j]!=0:
                        accu_t_depth = accu_t_depth + npDepth[i][j]
                        tree_depth = np.append(tree_depth, npDepth[i][j])
                        accu_t_amount = accu_t_amount + 1
            meanTree = statistics.mean(tree_depth)
            # medianTree = statistics.median(tree_depth)
            # stdevTree = statistics.stdev(tree_depth)
            # myUtils.write2CSV("statistics",[str(meanTree), str(medianTree), str(stdevTree)])

            cv2.putText(npTreeMask_c, #numpy array on which text is written
                        'mean: '+str(np.trunc(meanTree)), #text
                        (400,200+15), #position at which writing has to start
                        cv2.FONT_HERSHEY_SIMPLEX, #font family
                        0.9, #font size
                        (255,255,255), #font color
                        3) #font stroke
            
    else:
        print("no tree")

    npBinarFuse = cv2.addWeighted(npDepth_binary, 0.7, npHeight_binary, (0.4), 0.0)
    npBinarFuse = cv2.cvtColor(npBinarFuse, cv2.COLOR_GRAY2BGR)
    canvas = np.zeros_like(npTreeMask_c)
    
    p1 = np.hstack((npTreeMask_c,npBinarFuse))
    # p1 = cv2.cvtColor(p1, cv2.COLOR_GRAY2BGR)
    # p2 = np.hstack((npTreeMask_c, npDepthF_copy, canvas))
    # p3 = np.vstack((p1,p2))
    
    cv2.imwrite(file_path + str(file_index/10) + '.jpg', p1)
    # cv2.imwrite(file_path + str(file_index/10) + 't.jpg', npBinarFuse)
        # cv2.imshow('low pass', npTreeMask_c)
        # cv2.waitKey(1)
    # cv2.destroyAllWindows()

class Synchronize:
    def __init__(self):
        self.msgColor = None
        self.msgDepth = None
        self.flagColor = False
        self.flagDepth = False
    def colorIn(self, color):
        self.msgColor = color
        self.flagColor = True
    def depthIn(self, depth, index):
        self.msgDepth = depth
        self.flagDepth = True
        self.index = index
    def show(self):
        if (self.flagDepth and self.flagColor) == True:
            print("yes")
            self.imgColor = msg2CV(self.msgColor)
            self.imgDepth = msg2CV(self.msgDepth)
            # np.save(file_path + str(int(index/10))+'_'+str(int(index%10)), self.imgDepth)
            fnGroundSeg(self.imgColor, self.imgDepth, self.index)
        else: 
            print("not yet")

synchronizer = Synchronize()

index = 0

def cbDepth(msg):
    global index, file_path, synchronizer
    
    if index%2 == 0:
        print("receive depth!")
        # np.save(file_path + str(int(index/10)), image)
        synchronizer.depthIn(msg, index)
        synchronizer.show()
        # image = msg2CV(msg)
        # cv2.imshow('raw image', image)
        # cv2.waitKey(1)
        # fnGroundSeg(image, str(int(index/10)))
        

    index = index+1

def cbColor(msg):
    global file_path, synchronizer
    # print("receive color!")
    synchronizer.colorIn(msg)
    # if index%10 == 0:
        # colorImg = msg2CV(msg)
        # cv2.imwrite(file_path + str(int(index/10)) + '_c.jpg', pic12)

if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, cbDepth)
    subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()
