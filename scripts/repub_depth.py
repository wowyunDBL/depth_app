#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo
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
    print(msg.header.stamp)
    print(rospy.Time.now())
    print("-----------------")
    msg.header.stamp = rospy.Time.now()
    pubDepth.publish(msg)
    
def cbDepthInfo(msg):
    msg.header.stamp = rospy.Time.now()
    pubDepthInfo.publish(msg)

    
if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, cbDepth)
    subDepthInfo = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, cbDepthInfo)
    pubDepth = rospy.Publisher("/camera/aligned_depth_to_color/image_raw/republish",  Image, queue_size=10)
    pubDepthInfo = rospy.Publisher("/camera/aligned_depth_to_color/camera_info/republish",  CameraInfo, queue_size=10)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()