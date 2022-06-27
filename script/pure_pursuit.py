#!/usr/bin/env python
# coding: UTF-8

from __future__ import print_function
import imp

import rospy
import roslib
import numpy as np
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

def line_create(num=0, x1=0, y1=0, z1=0, x2=0, y2=0, z2=0, r=0, g=0, b=0, a=1.0, scale=0.3):

    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.id = num
    marker_data.action = Marker.SPHERE
    marker_data.type = Marker.CUBE

    marker_data.scale.x = 0.1
    marker_data.scale.y = 0.1
    marker_data.scale.z = 0.1
    marker_data.pose.position.x = 0
    marker_data.pose.position.y = 0
    marker_data.pose.position.z = 0
    marker_data.pose.orientation.x = 0
    marker_data.pose.orientation.y = 0
    marker_data.pose.orientation.z = 0
    marker_data.pose.orientation.w = 1
    marker_data.color.g = 1.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0

    return marker_data

class Pursuit:
    def __init__(self):
        self.auto_flag = False

        self.pub_map_marker = rospy.Publisher('path_marker', MarkerArray, queue_size=10)

        loot_dir = roslib.packages.get_pkg_dir("robocon_slam")
        self.path = np.loadtxt(loot_dir+"/path/path.csv", delimiter=',')
        self.map_marker = MarkerArray()
        for i in range(self.path.shape[0]):
            self.map_marker.markers.append(line_create(i,
                                                       self.path[i, 0],
                                                       self.path[i, 1],
                                                       0,
                                                       self.path[i, 2],
                                                       self.path[i, 3],
                                                       0,
                                                       1, 1, 1, 1
                                                       ))

    def flag_CB(self, data):
        self.auto_flag = data.data

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit', anonymous=True)
        Hz = 20
        r = rospy.Rate(Hz)
        mypursuit = Pursuit()

        rospy.Subscriber("auto_flag", Bool, mypursuit.flag_CB)
        print(mypursuit.path)
        try:
            while not rospy.is_shutdown():
                
                r.sleep()
        except KeyboardInterrupt:
            pass
    except rospy.ROSInterruptException:
        pass