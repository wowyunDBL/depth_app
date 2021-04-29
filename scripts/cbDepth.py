#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/p_1_45/"

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

def fnGroundSeg(npDepth, file_index):
    '''filter out 6m'''
    npDepthF = cv2.convertScaleAbs(npDepth, alpha=0.04) # 6m
    npDepthF_color = cv2.applyColorMap(npDepthF, cv2.COLORMAP_JET)
    # cv2.imshow("filtered", npDepthF_color)
    # cv2.waitKey(500)

    '''depth segmentation'''
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
    npPointY = npPointY.dot(npDepth)/ fy_d * (-1) + 360
    npPointY = npPointY.astype('float16')

    '''depth segmentation'''
    npHeight = np.copy(npPointY)
    height,width = npHeight.shape
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

    '''ground height statistics'''
    ground_height = np.array([])
    layer = 20
    ground_height = npHeight[height-layer*10: height][npHeight[height-layer*10: height]!=360]
    ground_height = ground_height.astype('float64')

    meanGrass = statistics.mean(ground_height)
    medianGrass = statistics.median(ground_height)
    stdevGrass = statistics.stdev(ground_height)

    myUtils.write2CSV(1,[str(meanGrass), str(medianGrass), str(stdevGrass), 
                            str(np.min(ground_height)), str(np.max(ground_height))])

    thres = 80 #medianGrass

    '''visualize statistics results'''
    npDepth_seg_c = np.copy(npDepth_seg)
    cv2.putText(npDepth_seg_c, #numpy array on which text is written
                'mean: '+str(np.trunc(meanGrass)), #text
                (200,200+15), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npDepth_seg_c, #numpy array on which text is written
                'median: '+str(medianGrass), #text
                (200,200+45), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npDepth_seg_c, #numpy array on which text is written
                'stdev: '+str(np.trunc(stdevGrass)), #text
                (200,200+75), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npDepth_seg_c, #numpy array on which text is written
                'min: '+str(np.min(ground_height)), #text
                (200,200+105), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.putText(npDepth_seg_c, #numpy array on which text is written
                'max: '+str(np.max(ground_height)), #text
                (200,200+135), #position at which writing has to start
                cv2.FONT_HERSHEY_SIMPLEX, #font family
                0.9, #font size
                (255,255,255), #font color
                3) #font stroke
    cv2.line(npDepth_seg_c, (0,height-layer*10),(640,height-layer*10),(255,255,255),3) 
    # cv2.imshow('ground height statistics',npDepth_seg_c)
    # cv2.waitKey(500)

    '''plot hog'''
    HOG_height = np.zeros(2000)
    for i in range(height-layer*10, height):
        for j in range(width):
            if npHeight[i][j]!=360:
                index = int(npHeight[i][j])+1000
                if index<0:
                    index = 0
                HOG_height[index] = HOG_height[index]+1
    HOG_height = HOG_height.astype('str')
    myUtils.write2CSV("histogram",HOG_height.tolist())

    '''threshold the ground'''
    npHeight = npHeight.astype('float32')
    ret, npHeight = cv2.threshold(npHeight, thres, 255, cv2.THRESH_BINARY_INV) # >thres = 0
    npHeight_c = cv2.cvtColor(npHeight,cv2.COLOR_GRAY2RGB)
    cv2.line(npHeight, (0,height-layer*10),(640,height-layer*10),(0,0,0),3)
    npHeight_c = npHeight_c.astype('uint8')
    
    pic11 = np.hstack((npHeight_seg, npDepth_seg_c))
    pic12 = np.hstack((pic11, npHeight_c))
    # pic13 = np.hstack((pic12, npHeight_seg))
    # pic21 = np.hstack((npHeight_seg, npHeight_c))
    # pic3 = np.vstack((pic21, pic11))

    cv2.imwrite(file_path + file_index + '.jpg', pic12)
    cv2.imshow('binarize', pic12)
    cv2.waitKey(1)
    # cv2.destroyAllWindows()

index = 0

def cbDepth(msg):
    global index, file_path
    if index%2 == 0:
        image = msg2CV(msg)
        cv2.imshow('raw image', image)
        cv2.waitKey(1)
        np.save(file_path + str(int(index/2)), image)
        fnGroundSeg(image, str(int(index/2)))

    index = index+1
globalCount = 0
def cbColor(msg):
    global index, file_path, globalCount
    globalCount = globalCount+1
    print(globalCount)
    if index%2 == 0:
        colorImg = msg2CV(msg)
        np.save(file_path + str(int(index/2))+'_c', colorImg)

if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, cbDepth)
    subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    print("successfully initialized!")
    print("Python version: ",sys.version)
    rospy.spin()
