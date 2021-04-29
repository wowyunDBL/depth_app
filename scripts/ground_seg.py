import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/"
#     sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
else:
    file_path = r"G:/我的雲端硬碟/0327_align_depth/"

'''math tool'''
import csv
import numpy as np
import time
np.set_printoptions(threshold=sys.maxsize)

'''plot tool'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import matplotlib.ticker as ticker

'''image tool'''
import cv2
import statistics # as sta
import myUtils

'''load file'''
npDepth = []
npDepth = np.load(file_path+"p_1_far.npy")
npBGR = []
# npBGR = np.load(file_path+"p_1_far_c.npy")
# cv2.imshow("color", npBGR)
# cv2.waitKey(500)

'''filter out 6m'''
npDepthF = cv2.convertScaleAbs(npDepth, alpha=0.04) # 6m
npDepthF_color = cv2.applyColorMap(npDepthF, cv2.COLORMAP_JET)
cv2.imshow("filtered", npDepthF_color)
cv2.waitKey(500)

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
cv2.imshow('depth segmentation', npDepth_seg)
cv2.waitKey(500)

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
cv2.imshow('height segmentation', npHeight_seg)
cv2.waitKey(500)

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
cv2.imshow('ground height statistics',npDepth_seg_c)
cv2.waitKey(500)

'''plot hog'''
HOG_height = np.zeros(2000)
for i in range(height-layer*10, height):
    for j in range(width):
        if npHeight[i][j]!=360:
            index = int(npHeight[i][j])+1000
            if index<0:
                print("smaller than 0")
            HOG_height[index] = HOG_height[index]+1
fig = plt.subplots(figsize=(13,5))
plt.plot(HOG_height, linewidth=1.0, color='#008b8b', label='right')
plt.axvline(thres, color='r')
plt.title('Histogram of height')
plt.show()

HOG_height = HOG_height.astype('str')
myUtils.write2CSV("histogram",HOG_height.tolist())


'''threshold the ground'''
npHeight = npHeight.astype('float32')
ret, npHeight = cv2.threshold(npHeight, thres, 255, cv2.THRESH_BINARY_INV) # >thres = 0
cv2.line(npHeight, (0,height-layer*10),(640,height-layer*10),(0,0,0),3)
cv2.imshow('binarize', npHeight)
cv2.waitKey(500)
pic11 = np.hstack((npDepthF_color, npHeight_seg))
    pic12 = np.hstack((pic11, npHeight))
cv2.destroyAllWindows()