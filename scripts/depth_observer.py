#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from depth_app.msg import ObsInformation

import transforms3d
import tf
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    file_path = "/home/ncslaber/109-2/simulation/"

'''math tool'''
import csv
import numpy as np
import time
import statistics
import math
np.set_printoptions(threshold=sys.maxsize)

'''image tool'''
import cv2
#import myUtils
from cv_bridge import CvBridge, CvBridgeError
import utm

'''plot tool'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm, transforms
from matplotlib.ticker import LinearLocator
import matplotlib.ticker as ticker
# from pyproj import Proj

'''trunk center tracker'''
from collections import deque
#from pyimagesearch.centroidtracker_mine import CentroidTracker

avoidance_dist = 2.5
flag = False
param_model = np.zeros((320,640))
count_init = 1

'''
PointX_g = []
PointY_g = []
tx_g = np.array([])
ty_g = np.array([])

for i in range(24):#29
    for j in range(0,10,2):
        # file_path = "/home/anny/109-2/0420_treeExperiment/npy/2_gps_npy/np_"
        PointX, PointY, tx, ty = np.load(file_path+str(i)+"_"+str(j)+".npy", allow_pickle=True)
        PointX_g.append(PointX)
        PointY_g.append(PointY)
        tx_g = np.append(tx_g,tx)
        ty_g = np.append(ty_g,ty)
'''

def msg2CV(msg):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #image = np.resize(image, (480,640))
        return image
    except CvBridgeError as e:
        print(e)
        return

def fnFilterDepth(npDepth):
    npDepth_binary = np.copy(npDepth)
    npDepth_binary = npDepth_binary.astype('float32')

    ret, npDepth_binary1 = cv2.threshold(npDepth_binary, 4500, 255, cv2.THRESH_BINARY_INV)
    ret, npDepth_binary2 = cv2.threshold(npDepth_binary, 500, 255, cv2.THRESH_BINARY)
    npDepth_binary1 = npDepth_binary1.astype('uint8')
    npDepth_binary2 = npDepth_binary2.astype('uint8')

    npDepth_binary = cv2.bitwise_and(npDepth_binary1, npDepth_binary2)
    npDepth_binary = npDepth_binary.astype('uint8')
    return npDepth_binary

def fnFilterHeight(npHeight):
    global flag, count_init, param_model, file_path
    ret, npHeight_binary = cv2.threshold(npHeight, 150, 255, cv2.THRESH_BINARY) # below 100mm
    npHeight_binary = npHeight_binary.astype('uint8')
    
    if flag:
        alpha=0.2
        param_model = (1-alpha)*param_model+alpha*npHeight_binary
        param_model = param_model.astype('uint8')
        moving_avg = np.zeros_like(param_model)
        moving_avg[param_model<=130]=int(0)
        moving_avg[param_model>130]=int(255)

    else:
        if count_init == 5:
            flag = True
            param_model = param_model/5
            moving_avg = np.zeros_like(param_model)
            moving_avg[param_model<=130]=int(0)
            moving_avg[param_model>130]=int(255)
            moving_avg = moving_avg.astype('uint8')
            print("BD: initialized param ground!")

        elif count_init < 5:
            moving_avg = np.copy(npHeight_binary)
            param_model = param_model + npHeight_binary
            count_init = count_init+1
            print("BD: not yet initial: "+str(count_init)+" frames")

    return moving_avg, npHeight_binary

def fnWorldCoord(npDepth):
    cx_d = 328 #424
    cy_d = 241 #241
    fx_d = 617 #424
    fy_d = 617 #424
    layer = 80 # middle point cloud
    npPointX = np.asarray(range(640))-cx_d
    npPointX = np.diag(npPointX)
    npPointX = npDepth.dot(npPointX)/ fx_d * (-1)
    # npPointX_2D = npPointX*npDepth[layer] / fx_d 
    # npPointX_2D = npPointX_2D.astype('float16')

    npPointY = np.asarray(range(320))-cy_d+160
    npPointY = np.diag(npPointY)
    theta = 0/180*np.pi
    npPointY = npPointY.dot(npDepth)/ fy_d * (-1) 
    npPointY = npPointY*np.cos(theta) + 360 # + npDepth * np.sin(theta)
    npPointY = npPointY.astype('float16')
    npPointZ = npDepth
    # npPointZ_2D = np.copy(npDepth[layer])
    # return npPointY, npPointX_2D, npPointZ_2D
    return npPointY, npPointX, npPointZ

flagChangeMode = False
def fnEasyAvoidance(npColor, npDepth, file_index, trans,rot):
    global avoidance_dist, count_detection, flagChangeMode
    
    height,width = npDepth.shape
    
    np.save(file_path+"treeDepth_",npDepth)
    
    '''filter out 3m'''
    npDepthROI = npDepth[160:]
    npDepth_binary = fnFilterDepth(npDepthROI)

    '''world coordinate'''
    npPointY, npPointX, npPointZ = fnWorldCoord(npDepthROI)

    '''filter out 5mm'''
    npHeight = np.copy(npPointY)
    npHeight = npHeight.astype('float32')
    moving_avg, npHeight_binary = fnFilterHeight(npHeight)
    npTreeMask = cv2.bitwise_and(npDepth_binary, moving_avg)
    np.save(file_path+"treeMask_",npTreeMask)
    '''
    index = np.array(range(640))
    npPointX_2D = npPointX[80]
    npPointZ_2D = npPointZ[80]
    base = np.ones(640)
    npPointXZ = np.vstack((npPointX,npPointZ)) 
    npPointXZ = npPointXZ[:,npPointXZ[1,:]!=0]
    npPointXZ = npPointXZ.reshape(3,-1)
    npPointXZ = npPointXZ[:,npPointXZ[1,:]<4500]
    npPointXZ = npPointXZ.reshape(3,-1)
    npPointXY = np.vstack((npPointXZ[1]/1000, -npPointXZ[0]/1000, npPointXZ[2]))# notice I transform coordinate

    
    y_y, p_p, r_r = transforms3d.euler.quat2euler([rot[0], rot[1], rot[2], rot[3]])
    tx = trans[0] # trans = (x,y,z)
    ty = trans[1]
    rotationM = np.array([[np.cos(y_y),-np.sin(y_y), tx],[np.sin(y_y),np.cos(y_y),ty],[0,0,1]])
    
    # npPointXY = np.dot(rot_m, npPointXY)
    npPointXY = np.dot(rotationM, npPointXY)
    np.save(file_path+"np_"+str(file_index/10) +'_'+str(int(file_index%10)), (npPointXY[0], npPointXY[1], tx, ty))
    '''
    if len(npPointZ[npTreeMask==255]) == 0:
        print("not observe any obj:")
    else:
        obs_min_dist = np.min(npPointZ[npTreeMask==255])
        obs_width = np.max(npPointX[npTreeMask==255]) - np.min(npPointX[npTreeMask==255])
        print("min_dist:", obs_min_dist)
        
        if obs_min_dist/1000 < avoidance_dist:
            
            print("detect obs! ")
            # count_detection += 1
            # if count_detection == 3:
            #     print("save map-------------------")
            #     build_map(npPointX, npPointZ, npTreeMask)
            if flagChangeMode == False:
                flagChangeMode = True
                msgMode = UInt8()
                msgMode.data = 3
                pub_control_mode.publish(msgMode)

            msgObs = ObsInformation()
            msgObs.obs_min_dist = obs_min_dist/1000
            msgObs.obs_width = obs_width
            pub_obs_info.publish(msgObs)
        else:
            flagChangeMode = False

count = 0
class Synchronize:
    def __init__(self):
        self.msgColor = None
        self.msgDepth = None
        self.stamp = None
        self.flagColor = False
        self.flagDepth = False
        self.listener = tf.TransformListener()
    def colorIn(self, color):
        self.msgColor = color
        self.flagColor = True
    def depthIn(self, depth, index):
        self.msgDepth = depth
        self.flagDepth = True
        self.index = index
        # self.stamp = depth.header.stamp

    def show(self):
        if (self.flagDepth and self.flagColor) == True:
            print("Enter fnEasyAvoidance Times: "+str(count))
            # print("reveive both color and depth")
            self.imgColor = msg2CV(self.msgColor)
            self.imgDepth = msg2CV(self.msgDepth)
            # np.save(file_path + 'npyD_' + str(int(index/10))+'_'+str(int(index%10)), self.imgDepth)
            #np.save(file_path + 'npyC_'+ str(int(index/10))+'_'+str(int(index%10)), self.imgColor)
            # cv2.imwrite(file_path + 'c_'+str(int(index/10))+'_'+str(int(index%10)) + '.jpg', self.imgColor)
            
            try:
                (trans,rot) = self.listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0)) # observer_frame, goal_frame
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("not receive tf...")
            

            #fnGroundSeg(self.imgColor, self.imgDepth, self.index, self.stamp, self.gps2D, trans,rot)
            fnEasyAvoidance(self.imgColor, self.imgDepth, self.index, trans,rot)
        else: 
            print("wait for color or depth")

rospy.init_node("depthHandler", anonymous=True)
synchronizer = Synchronize()

index = 0

def cbDepth(msg):
    global index, file_path, synchronizer, count
    flag_start = True
    if index%10 == 0:
        synchronizer.depthIn(msg, index)
        synchronizer.show()
        # print("count_callback: "+str(count))
        count += 1
    index = index+1

def cbColor(msg):
    global file_path, synchronizer
    synchronizer.colorIn(msg)

if __name__ == "__main__":
    
    subDepth = rospy.Subscriber("/camera/depth/image_raw", Image, cbDepth)
    subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    pub_control_mode = rospy.Publisher('/control_mode', UInt8, queue_size=10)
    pub_obs_info = rospy.Publisher('/obs_info', ObsInformation, queue_size=10)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()
    
    cv2.destroyAllWindows()