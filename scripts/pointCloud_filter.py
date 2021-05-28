#!urs/bin/env python

from sensor_msgs.msg import PointCloud2, Image

pubPL_filtered = None

def cbPL(msgPL):
    global pubPL_filtered
    print("successfully callback PointCloud!")
    pubPL_filtered.publish(msgPL)


if __name__ == "__main__":
    global pubPL_filtered
    rospy.init_node("point_cloud_filter", anonymous=True)
    #subPL = rospy.Subscriber("/camera/modified/color_image", Image, cbIMG)
    subPL = rospy.Subscriber("/camera/depth/color/points", PointCloud2, cbPL)
    #subPL = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", PointCloud2, cbIMG)
    
    pubPL_filtered = rospy.Publisher("/camera/modified/filtered_points", PointCloud2)
    print ("initialized!")
