#!usr/bin/env python
'''ros utils'''
import rospy
import tf

if __name__=='__main__':
    rospy.init_node("TFrepub", anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,quar) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            br.sendTransform(trans, quar, rospy.Time.now(),
                                    '/base_footprint',
                                    '/odom')
            print(trans, quar)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        rate.sleep()