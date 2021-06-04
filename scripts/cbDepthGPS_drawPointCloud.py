#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
import transforms3d
import tf
import sys
if sys.platform.startswith('linux'): # or win
    print("in linux")
    # file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/p_1_45_center_try/"
    file_path = "/home/anny/109-2/0420_treeExperiment/npy/2_gps_npy/"
    # print(sys.path)
# import message_filters

'''math tool'''
import csv
import numpy as np
import time
import statistics
import math
np.set_printoptions(threshold=sys.maxsize)

'''image tool'''
import cv2
import myUtils
from cv_bridge import CvBridge, CvBridgeError
import utm

'''plot tool'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm, transforms
from matplotlib.ticker import LinearLocator
import matplotlib.ticker as ticker
from pyproj import Proj

'''trunk center tracker'''
from collections import deque
from pyimagesearch.centroidtracker_mine import CentroidTracker
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
param_model = np.zeros((320,640))
count_init = 1

centroidTracker = CentroidTracker()  
(height, width) = (None, None)

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

def fnFilterHeight(npHeight, file_index):
    global flag, count_init, param_model, file_path
    ret, npHeight_binary = cv2.threshold(npHeight, 100, 255, cv2.THRESH_BINARY) # below 100mm
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
    # npPointX = np.diag(npPointX)
    # npPointX = npDepth.dot(npPointX)/ fx_d * (-1)
    npPointX_2D = npPointX*npDepth[layer] / fx_d 
    npPointX_2D = npPointX_2D.astype('float16')

    npPointY = np.asarray(range(320))-cy_d+160
    npPointY = np.diag(npPointY)
    theta = 0/180*np.pi
    npPointY = npPointY.dot(npDepth)/ fy_d * (-1) 
    npPointY = npPointY*np.cos(theta) + 360 # + npDepth * np.sin(theta)
    npPointY = npPointY.astype('float16')
    npPointZ_2D = np.copy(npDepth[layer])
    return npPointY, npPointX_2D, npPointZ_2D

def fnMasking(npTreeMask):
    global height, width
    mask = np.zeros_like(npTreeMask)
    region_of_interest_vertices = [(0,height/3),(0, height),(width, height),(width,height/3)]
    cv2.fillPoly(mask, np.array([region_of_interest_vertices], np.int32), 255)
    npTreeMask = cv2.bitwise_and(npTreeMask, mask)
    return npTreeMask

def fnAvgDepth(npTreeMask,npDepth, x,y,w,h):
    accu_t_depth = 0
    # tree_depth = np.array([])
    accu_t_amount = 0
    for i in range(y, y+h):
        for j in range(x, x+w):
            if npTreeMask[i][j]!=0:
                accu_t_depth = accu_t_depth + npDepth[i][j]
                # tree_depth = np.append(tree_depth, npDepth[i][j])
                accu_t_amount = accu_t_amount + 1
    # meanTree = statistics.mean(tree_depth)
    # medianTree = statistics.median(tree_depth)
    # stdevTree = statistics.stdev(tree_depth)
    # myUtils.write2CSV("statistics",[str(meanTree), str(medianTree), str(stdevTree)])
    meanTree = accu_t_depth/accu_t_amount
    return meanTree

def saveCV(npDepth_binary, npHeight_binary, moving_avg, npTreeMask_c, npColor, file_index):
    global file_path
    npBinarFuse = cv2.addWeighted(npDepth_binary, 0.7, moving_avg, (0.4), 0.0)
    # npBinarFuse = cv2.cvtColor(npBinarFuse, cv2.COLOR_GRAY2BGR)
        # canvas = np.zeros_like(npTreeMask_c)
        
        # p1 = np.hstack((npTreeMask_c,npBinarFuse))
    # p1 = cv2.cvtColor(p1, cv2.COLOR_GRAY2BGR)
    # p2 = np.hstack((npTreeMask_c, npDepthF_copy, canvas))
    # p3 = np.vstack((p1,p2))
    # cv2.imwrite(file_path + 'trunc_'+str(file_index/10) +'_'+str(int(index%10))+ '.jpg', npTreeMask_c)
    # cv2.imwrite(file_path + 'binary_'+str(file_index/10) +'_'+str(int(index%10))+ '.jpg', npBinarFuse)

    npTreeMask_cc = np.vstack((np.zeros((160,640, 3), dtype='uint8'),npTreeMask_c))
    npTreeMask_cc = np.hstack((npTreeMask_cc, npColor))
    cv2.imwrite(file_path + 'result_'+str(file_index/10) +'_'+str(int(file_index%10))+ '.jpg', npColor)
    cv2.imwrite(file_path + 'Cstack_'+str(file_index/10) +'_'+str(int(file_index%10))+ '.jpg', npTreeMask_cc)

    fuse = np.hstack((npDepth_binary, npHeight_binary, moving_avg, npBinarFuse))
    cv2.imwrite(file_path + 'Mstack_'+str(file_index/10) +'_'+str(int(file_index%10))+ '.jpg', fuse)

initial_GPS = False
GPS_x = None
GPS_y = None

def fnGroundSeg(npColor, npDepth, file_index, stamp, gps2D):
    global flag, param_model, count_init, height,width, file_path
    global CentroidTracker #,H,W
    global initial_GPS, GPS_x, GPS_y

    listener = tf.TransformListener()
    
    rects = []
    height,width = npDepth.shape
    # npColort = npColor[160:]
    '''filter out 3m'''
    # start_time = time.time()
    npDepthROI = npDepth[160:]
    # npDepth_binary = fnFilterDepth(npDepthROI)
    # print("----------------filter depth: "+str(time.time()-start_time))

    '''world coordinate'''
    # start_time = time.time()
    npPointY, npPointX_2D, npPointZ_2D = fnWorldCoord(npDepthROI)
    # print("----------------world coord: "+str(time.time()-start_time))

    index = np.array(range(640))
    base = np.ones(640)
    npPointXZ = np.vstack((npPointX_2D,npPointZ_2D,base)) # notice I transform coordinate
    npPointXZ = npPointXZ[:,npPointXZ[1,:]!=0]
    npPointXZ = npPointXZ.reshape(3,-1)
    npPointXZ = npPointXZ[:,npPointXZ[1,:]<4500]
    npPointXZ = npPointXZ.reshape(3,-1)
    # npPointXZ = npPointXZ/1000
    npPointXY = np.vstack((npPointXZ[1]/1000, -npPointXZ[0]/1000, npPointXZ[2]))

    (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
    # rot_m = transforms3d.quaternions.quat2mat([rot[0], rot[1], rot[2], rot[3]])
    y_y, p_p, r_r = transforms3d.euler.quat2euler([rot[0], rot[1], rot[2], rot[3]])
    tx = trans[0] # trans = (x,y,z)
    ty = trans[1]
    rotationM = np.array([[np.cos(y_y),-np.sin(y_y), tx],[np.sin(y_y),np.cos(y_y),ty],[0,0,1]])
    #rotationM = np.array([[1,0,tx],[0,1,ty],[0,0,1]])
    # print(tx,ty)
    # npPointXZ = np.linalg.inv(rot_m).dot(npPointXZ)
    # print(npPointXY)
    # print(npPointXY.shape)
    npPointXY = np.dot(rotationM, npPointXY)
    # print(npPointXY)
    '''ROI
    # npTreeMask = fnMasking(npTreeMask)
    
    npTreeMask_c = cv2.cvtColor(npTreeMask, cv2.COLOR_GRAY2BGR)
    npTreeMask_c = cv2.addWeighted(npTreeMask_c, 0.4, npColort, (0.6), 0.0)'''
    
    # npDepthF_copy = np.copy(npDepthF)
    # npDepthF_copy = cv2.cvtColor(npDepthF_copy, cv2.COLOR_GRAY2BGR)

    if initial_GPS is False:
        GPS_x = gps2D[0]
        GPS_Y = gps2D[1]
        initial_GPS = True
    
    np.save(file_path+"np_"+str(file_index/10) +'_'+str(int(file_index%10)), (npPointXY[0], npPointXY[1], tx, ty))
    
    textStamp = stamp.secs + stamp.nsecs * 1e-9
    fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
    plt.grid(True)
    base = plt.gca().transData
    rotation = transforms.Affine2D().rotate_deg(135)#92
    # textGPS = "["+str(math.trunc(gps2D[0]*1000%1e6))+", "+str(math.trunc(gps2D[1]*1000%1e6))+"]"
    # plt.text(gps2D[0]+0.1-GPS_x, gps2D[1]+0.1-GPS_y, textGPS, transform=rotation + base,fontsize=22)
    # ax.text(gps2D[0]-0.1, gps2D[1]+0.1, str(math.trunc(i/5/60))+":"+str(math.trunc(i/5%60)), fontsize=14, transform = rot + base)
    '''
    textBase_footprint = "["+str(math.trunc(tx*1000))+", "+str(math.trunc(ty*1000))+"]"
    plt.text(tx+0.1, ty+0.1, textBase_footprint, transform=rotation + base,fontsize=22)
    
    plt.scatter(npPointXY[0], npPointXY[1], c='b', transform=rotation + base)
    # plt.scatter(gps2D[0]-GPS_x, gps2D[1]-GPS_y, c='r',transform=rotation + base, s=200, marker='*')
    plt.scatter(tx, ty, c='r',transform=rotation + base, s=200, marker='*')
    plt.title(str(textStamp), fontsize = 25)
    plt.xlim((3, 9))
    plt.ylim((6, 20))
    # plt.xlim((-2778360, -2778335))
    # plt.ylim((256076, 256088))
        # plt.xlim((352882,352898))
        # plt.ylim((2767708,2767718))
    ax.ticklabel_format(useOffset=False, style='sci')
    plt.savefig(file_path+"tree_and_gps"+str(file_index/10) +'_'+str(int(file_index%10))+".png", dpi=100)'''
    # np.save(file_path + 'npyXZ_' + str(int(file_index/10))+'_'+str(int(file_index%10)), npPointXZ)
    # np.save(file_path + 'npyGPS_' + str(int(file_index/10))+'_'+str(int(file_index%10)), gps2D)
        # plt.scatter(gps2D[0], gps2D[1], c='r',transform=rot + base, s=30)
        # plt.text(gps2D[0]+150, gps2D[1]+150, str(gps2D), transform=rot + base,fontsize=22)
        # plt.savefig(file_path+"only_gps"+str(file_index/10) +'_'+str(int(file_index%10))+".png")
    # start_time = time.time()
    # saveCV(npDepth_binary, npHeight_binary, moving_avg, npTreeMask_c, npColor, file_index)
    # print("----------------saving file: "+str(time.time()-start_time))
    
        # cv2.imshow('low pass', npTreeMask_c)
        # cv2.waitKey(1)
    # cv2.destroyAllWindows()

class Synchronize:
    def __init__(self):
        self.msgColor = None
        self.msgDepth = None
        self.flagGPS = None
        self.gps2D = None
        self.stamp = None
        self.flagColor = False
        self.flagDepth = False
    def colorIn(self, color):
        self.msgColor = color
        self.flagColor = True
    def depthIn(self, depth, index):
        self.msgDepth = depth
        self.flagDepth = True
        self.index = index
        # self.stamp = depth.header.stamp
    def gpsIn(self, utmX, utmY, gps):
        self.gps2D = np.array([utmX, utmY])
        self.stamp = gps.header.stamp
        self.flagGPS = True

    def show(self):
        if (self.flagDepth and self.flagColor and self.flagGPS) == True:
            # print("reveive both color and depth")
            self.imgColor = msg2CV(self.msgColor)
            self.imgDepth = msg2CV(self.msgDepth)
            # np.save(file_path + 'npyD_' + str(int(index/10))+'_'+str(int(index%10)), self.imgDepth)
            # np.save(file_path + 'npyC_'+ str(int(index/10))+'_'+str(int(index%10)), self.imgColor)
            #cv2.imwrite(file_path + 'c_'+str(int(index/10))+'_'+str(int(index%10)) + '.jpg', self.imgColor)
            fnGroundSeg(self.imgColor, self.imgDepth, self.index, self.stamp, self.gps2D)
        else: 
            print("wait color")

synchronizer = Synchronize()
count = 0
index = 0

def cbDepth(msg):
    global index, file_path, synchronizer, count
    
    if index%2 == 0:
            # print("receive depth!")
            # np.save(file_path + str(int(index/10)), image)
        synchronizer.depthIn(msg, index)
        synchronizer.show()
            # image = msg2CV(msg)
            # cv2.imshow('raw image', image)
            # cv2.waitKey(1)
            # fnGroundSeg(image, str(int(index/10)))
        print("count_callback: "+str(count))
        count += 1
    index = index+1
    

def cbColor(msg):
    global file_path, synchronizer
    # print("receive color!")
    synchronizer.colorIn(msg)
    
def cbGPS(msg):
    _, _, zone, _ = utm.from_latlon(msg.latitude, msg.longitude)
    proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
    utmX, utmY = proj(msg.longitude, msg.latitude)
    # print("cb GPS!")
    synchronizer.gpsIn(utmX, utmY, msg)


if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, cbDepth)
    subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    subGPS = rospy.Subscriber("/navsat/fix", NavSatFix, cbGPS)
    # subIMU = rospy.Subscriber("/navsat/fix", NavSatFix, cbGPS)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()