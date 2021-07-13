#! /usr/bin/env python

'''ros utils'''
import rospy
import tf

if __name__=='__main__':
    rospy.init_node("TFrepub", anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            (trans,quar) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            br.sendTransform(trans, quar, rospy.Time.now(),
                                    '/base_link_yun',
                                    '/odom_yun')
            (trans1,quar1) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
            br.sendTransform(trans1, quar1, rospy.Time.now(),
                                    '/odom_yun',
                                    '/map_yun')
            #print(rospy.Time.now())
            #print(rospy.Time(0))
            #print("----------------")

            #(trans1,quar1) = listener.lookupTransform('/base_footprint', '/camera_link', rospy.Time(0))
            #br.sendTransform(trans1, quar1, rospy.Time.now(),
            #                        '/camera_link',
            #                        '/base_footprint_yun')
            # print(trans, quar)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        rate.sleep()
