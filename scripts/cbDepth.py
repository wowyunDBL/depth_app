#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    file_path = "/home/ncslaber/109-2/210816_NTU_half/"
    # print(sys.path)
# import message_filters

'''math tool'''
import csv
import numpy as np
import time
np.set_printoptions(threshold=sys.maxsize)

'''image tool'''
import cv2
import statistics # as sta
import myUtils
from cv_bridge import CvBridge, CvBridgeError

'''plot tool'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.ticker as ticker

def msg2CV(msg):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return image
    except CvBridgeError as e:
        print(e)
        return

def fnGroundSeg(npColor, npDepth, file_index):
    '''filter out 6m'''
    height,width = npDepth.shape
    npDepthF = cv2.convertScaleAbs(npDepth, alpha=0.085) # 6m
    # npDepthF_color = cv2.applyColorMap(npDepthF, cv2.COLORMAP_JET)
    # cv2.imshow("filtered", npDepthF_color)
    # cv2.waitKey(500)

    npDepth_binary = np.copy(npDepth)
    npDepth_binary = npDepth_binary.astype('float32')

    ret, npDepth_binary1 = cv2.threshold(npDepth_binary, 3000, 255, cv2.THRESH_BINARY_INV)
    ret, npDepth_binary2 = cv2.threshold(npDepth_binary, 1500, 255, cv2.THRESH_BINARY)
    npDepth_binary1 = npDepth_binary1.astype('uint8')
    npDepth_binary2 = npDepth_binary2.astype('uint8')

    npDepth_binary = cv2.bitwise_and(npDepth_binary1, npDepth_binary2)
    npDepth_binary = npDepth_binary.astype('uint8')

    '''depth segmentation
    color_seq = {'brown': (0,130,210), 'red':(0,0,255),'yellow':(22,220,220),
                    'green':(0,255,0),'blue':(255,0,0),'black':(0,0,0)}
    npDepth_seg = np.zeros((npDepth.shape[0],npDepth.shape[1],3))

    npDepth_seg[npDepth<1500]=color_seq['blue']
    npDepth_seg[np.logical_and(npDepth<2000,npDepth>1500)]=color_seq['green']
    npDepth_seg[np.logical_and(npDepth<3000,npDepth>2000)]=color_seq['yellow']
    npDepth_seg[np.logical_and(npDepth<6000,npDepth>3000)]=color_seq['red']
    npDepth_seg[npDepth>6000]=color_seq['brown']
    npDepth_seg[npDepth==0]=color_seq['black']

    npDepth_seg = npDepth_seg.astype('uint8')
    '''
    # cv2.imshow('depth segmentation', npDepth_seg)
    # cv2.waitKey(500)

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

    npHeight_binary = np.copy(npPointY)
    npHeight_binary = npHeight_binary.astype('float32')

    ret, npHeight_binary = cv2.threshold(npHeight_binary, 5, 255, cv2.THRESH_BINARY)
    npHeight_binary = npHeight_binary.astype('uint8')

    npTreeMask = cv2.bitwise_and(npDepth_binary, npHeight_binary)
    mask = np.zeros_like(npTreeMask)
    region_of_interest_vertices = [(0,height/2),(0, height),(width, height),(width,height/2)]
    cv2.fillPoly(mask, np.array([region_of_interest_vertices], np.int32), 255)
    npTreeMask = cv2.bitwise_and(npTreeMask, mask)
    
    npTreeMask_c = cv2.cvtColor(npTreeMask, cv2.COLOR_GRAY2BGR)
    npTreeMask_c = cv2.addWeighted(npTreeMask_c, 0.5, npColor, (0.4), 0.0)

    contours, hierarchy = cv2.findContours(npTreeMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
    npDepthF_copy = np.copy(npDepthF)
    npDepthF_copy = cv2.cvtColor(npDepthF_copy, cv2.COLOR_GRAY2BGR)

    '''extract the only obj'''
    allObj_plot = np.zeros(npDepth.shape, dtype='uint8')
    moments = []
    index = []
    objSizeBound = 8000
    i = 0
    for ct in contours:
        obj_plot = np.zeros(npDepth.shape, dtype='uint8')
        cv2.fillPoly(obj_plot, [ct], 255)
    #     cv2.imshow('obj',obj_plot)
    #     cv2.waitKey(10)
        m = cv2.moments(ct)
        if m["m00"] > objSizeBound:
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
    else:
        print("no tree")

    npBinarFuse = cv2.addWeighted(npDepth_binary, 0.7, npHeight_binary, (0.4), 0.0)

    '''depth segmentation
    npHeight = np.copy(npPointY)
    
    color_seq = {'brown': (0,130,210), 'red':(0,0,255),'yellow':(22,220,220),
                    'green':(0,255,0),'blue':(255,0,0),'black':(0,0,0)}
    npHeight_seg = np.zeros((npHeight.shape[0],npHeight.shape[1],3))

    npHeight_seg[npHeight<-160]=color_seq['blue']
    npHeight_seg[np.logical_and(npHeight<-80,npHeight>-160)]=color_seq['green']
    npHeight_seg[np.logical_and(npHeight<0,npHeight>-80)]=color_seq['yellow']
    npHeight_seg[np.logical_and(npHeight<80,npHeight>0)]=color_seq['red']
    npHeight_seg[npHeight>80]=color_seq['brown']
    npHeight_seg[npHeight==360]=color_seq['black']

    npHeight_seg = npHeight_seg.astype('uint8')
    # cv2.imshow('height segmentation', npHeight_seg)
    # cv2.waitKey(500)
    '''

    '''ground height statistics
    ground_height = np.array([])
    layer = 20
    ground_height = npHeight[height-layer*10: height][npHeight[height-layer*10: height]!=360]
    ground_height = ground_height.astype('float64')

    meanGrass = statistics.mean(ground_height)
    medianGrass = statistics.median(ground_height)
    stdevGrass = statistics.stdev(ground_height)

    myUtils.write2CSV("statistics",[str(meanGrass), str(medianGrass), str(stdevGrass), 
                            str(np.min(ground_height)), str(np.max(ground_height))])
    '''

    thres = 5 #medianGrass

    '''visualize statistics results
    npHeight_seg_c = np.copy(npHeight_seg)
    cv2.putText(npHeight_seg_c, #numpy array on which text is written
                'mean: '+str(np.trunc(meanGrass)), #text
                (200,200+15), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npHeight_seg_c, #numpy array on which text is written
                'median: '+str(medianGrass), #text
                (200,200+45), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npHeight_seg_c, #numpy array on which text is written
                'stdev: '+str(np.trunc(stdevGrass)), #text
                (200,200+75), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npHeight_seg_c, #numpy array on which text is written
                'min: '+str(np.min(ground_height)), #text
                (200,200+105), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npHeight_seg_c, #numpy array on which text is written
                'max: '+str(np.max(ground_height)), #text
                (200,200+135), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.line(npHeight_seg_c, (0,height-layer*10),(640,height-layer*10),(255,255,255),3) 
    # cv2.imshow('ground height statistics',npHeight_seg_c)
    # cv2.waitKey(500)
    '''

    '''plot hog
    HOG_height = np.zeros(2000)
    for i in range(height-layer*10, height):
        for j in range(width):
            if (npHeight[i][j]!=360 or npHeight[i][j]<80):
                index = int(npHeight[i][j])+1000
                if index<0:
                    index = 0
                HOG_height[index] = HOG_height[index]+1
    HOG_height = HOG_height.astype('str')
    myUtils.write2CSV_column("histogram",HOG_height.tolist())
    '''

    '''threshold the ground
    npHeight = npHeight.astype('float32')
    ret, npHeight = cv2.threshold(npHeight, thres, 255, cv2.THRESH_BINARY_INV) # >thres = 0
    npHeight_c = cv2.cvtColor(npHeight,cv2.COLOR_GRAY2RGB)
    cv2.line(npHeight_c, (0,height-layer*10),(640,height-layer*10),(0,0,0),3)
    npHeight_c = npHeight_c.astype('uint8')
    
    alpha=0.8
    npColorMask = cv2.addWeighted(npColor, alpha, npHeight_c, (0.4), 0.0)
    '''
    # npColorMask = np.bitwise_or(npColor, npHeight_c)
    
    # pic11 = np.hstack((npDepthF_color, npDepth_seg))
    # pic12 = np.hstack((pic11, npHeight_c))
    # pic13 = np.hstack((pic12, npHeight_seg))
    # pic21 = np.hstack((npHeight_seg_c, npHeight_c))
    # pic31 = np.hstack((npColor, npColorMask))
    # pic3 = np.vstack((pic11, pic21))
    # pic4 = np.vstack((pic3, pic31))
    canvas = np.zeros_like(npTreeMask_c)
    
    p1 = np.hstack((npDepth_binary,npHeight_binary, npBinarFuse))
    p1 = cv2.cvtColor(p1, cv2.COLOR_GRAY2BGR)
    p2 = np.hstack((npTreeMask_c, npDepthF_copy, canvas))
    p3 = np.vstack((p1,p2))
    

    cv2.imwrite(file_path + str(file_index/10) + '.jpg', p3)
    # cv2.imshow('binarize', p3)
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
            np.save(file_path + "depth", self.imgDepth)
            np.save(file_path + "color", self.imgColor)
            # fnGroundSeg(self.imgColor, self.imgDepth, self.index)
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
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth)
    subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()
