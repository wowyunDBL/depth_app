#!usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import sys

def msg2CV(msg):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return image
    except CvBridgeError as e:
        print(e)
        return


def cbDepth(msg):
    print("callback!")
    image = msg2CV(msg)
    cv2.imshow('raw image', image)
    #cv2.circle(image,(0,0),15,(255,0,0),-1)
    cv2.waitKey(1)
    file_path='/home/ncslaber/109-2/tree_experiment/npy_depth/'
    np.save(file_path + 'p_1_middle', image)

def cbColor(msg):
    image = msg2CV(msg)
    file_path='/home/ncslaber/109-2/tree_experiment/npy_depth/'
    np.save(file_path + 'p_1_middle_c', image)

if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, cbDepth)
    subIntrinsic = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    print("successfully initialized!")
    print("Python version: ",sys.version)
    rospy.spin()
