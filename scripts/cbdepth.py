#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def msg2CV(msg):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return image
    except CvBridgeError as e:
        print(e)
        return

def cbDepth(msg):
    imageArray = msg2CV(msg)
    a=msg.data
    print(a.size)
    # print(type(imageArray[240][0]))
    # print(imageArray[240][0])
    # print("---------")
    # print(type(imageArray[240][1]))
    # print(imageArray[240][1])

if __name__ == "__main__":
    rospy.init_node("depthHandler", anonymous=True)
    subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth)
    # subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    print("successfully initialized!")
    # print("Python version: ",sys.version)

    rospy.spin()