#!/usr/bin/env python
# coding: UTF-8  
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((2.5, 2.5, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "base_link",
                         "map")
        rate.sleep()