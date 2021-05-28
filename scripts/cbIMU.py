#!usr/bin/env python
import rospy 
from sensor_msgs.msg import Imu
import transforms3d
import numpy as np

pitch = np.array([])

def cbImu(msg):
    global pitch
    y_y, p_p, r_r = transforms3d.euler.quat2euler([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    pitch = np.append(pitch, p_p)
    np.save(str("imu"),pitch)
    print("~")


if __name__=="__main__":
    rospy.init_node("imuHandler", anonymous=True)
    subDepth = rospy.Subscriber("/imu/data", Imu, cbImu)
    rospy.spin()
