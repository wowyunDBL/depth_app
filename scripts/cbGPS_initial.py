#!usr/bin/env python
'''ros utils'''
import rospy
from sensor_msgs.msg import NavSatFix

'''math tool'''
import csv
import numpy as np
import time
import statistics

count = 0
latList = []
lonList= []

def cbGPS(msg):
    global count, latList, lonList
    count += 1
    if count %5 == 0 and count <= 50:
        latList.append(msg.latitude)
        lonList.append(msg.longitude)
        print("cb GPS!")
        
        if count == 50:
            print("successfully initialized!")
            latList = np.asarray(latList)
            lonList = np.asarray(lonList)
            lat = statistics.mean(latList)
            lon = statistics.mean(lonList)
            with open('/home/ncslaber/lat_lon.csv', 'w') as csvfile: # or w
                writer = csv.writer(csvfile)
                writer.writerows([[lat],[lon]])
            print(len(lonList))


if __name__ == "__main__":
    rospy.init_node("initialGPS_node", anonymous=True)
    subGPS = rospy.Subscriber("/navsat/fix", NavSatFix, cbGPS)
    rospy.spin()
