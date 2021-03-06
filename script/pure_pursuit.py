#!/usr/bin/env python
# coding: UTF-8

from __future__ import print_function
import imp
from re import X

import rospy
import roslib
import tf
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

def line_create(num=0, x_=0, y_=0, z_=0, r_=1, g_=0, b_=0, a_=1.0, scale=0.1):

    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.id = num
    marker_data.action = Marker.ADD
    marker_data.type = Marker.SPHERE
    marker_data.lifetime = rospy.Duration()

    marker_data.scale.x = scale
    marker_data.scale.y = scale
    marker_data.scale.z = scale
    marker_data.pose.position.x = x_
    marker_data.pose.position.y = y_
    marker_data.pose.position.z = z_
    marker_data.pose.orientation.x = 0
    marker_data.pose.orientation.y = 0
    marker_data.pose.orientation.z = 0
    marker_data.pose.orientation.w = 1
    marker_data.color.r = r_
    marker_data.color.g = g_
    marker_data.color.b = b_
    marker_data.color.a = a_

    return marker_data

class Pursuit:
    def __init__(self):
        self.auto_flag = False
        self.pose = [0.0]*3
        self.vel = [0.0]*3
        self.cmd_vel = Twist()
        self.listener = tf.TransformListener()
        self.remarkable_point = [0.0]*2

        #pure_pursuitのパラメータ
        self.dlimit = 0.05
        self.vel_gain = 5
        self.max_vel = 1

        self.pub_path = rospy.Publisher('path_marker', MarkerArray, queue_size=10)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        loot_dir = roslib.packages.get_pkg_dir("robocon_slam")
        self.path = np.loadtxt(loot_dir+"/path/path.csv", delimiter=',')
        self.path_marker = MarkerArray()

    def flag_CB(self, data):
        self.auto_flag = data.data
        
    def run_pursuit(self, trans, rot):
        self.pose[0] = trans[0]
        self.pose[1] = trans[1]
        self.pose[2] = tf.transformations.euler_from_quaternion(rot)

        d = (self.path[:, 0] - self.pose[0])**2 + (self.path[:, 1] - self.pose[1])**2
        self.remarkable_point = np.argmin(d)

        if self.remarkable_point <= len(self.path)-2:
            if d[self.remarkable_point] < self.dlimit:
                self.remarkable_point += 1
            self.vel[0] = self.vel_gain*(self.path[self.remarkable_point, 0] - self.pose[0])
            self.vel[1] = self.vel_gain*(self.path[self.remarkable_point, 1] - self.pose[1])
            if self.vel[0] > self.max_vel:
                self.vel[0] = self.max_vel
            if self.vel[1] > self.max_vel:
                self.vel[1] = self.max_vel
            self.cmd_vel.linear.x = self.vel[0]
            self.cmd_vel.linear.y = self.vel[1]
        
        if self.auto_flag:
            self.pub_vel.publish(self.cmd_vel)

        for i in range(self.path.shape[0]):
            if i==self.remarkable_point:
                self.path_marker.markers.append(line_create(i,
                                                    self.path[i, 0],
                                                    self.path[i, 1],
                                                    0,
                                                    1,0,0,1,
                                                    0.1
                                                    ))
            else:
                self.path_marker.markers.append(line_create(i,
                                                    self.path[i, 0],
                                                    self.path[i, 1],
                                                    0,
                                                    0,1,0,1,
                                                    0.1
                                                    ))
        self.pub_path.publish(self.path_marker)


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
                try:
                    (trans, rot) = mypursuit.listener.lookupTransform('map', 'base_link', rospy.Time(0))
                    mypursuit.run_pursuit(trans, rot)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass
                r.sleep()
        except KeyboardInterrupt:
            pass
    except rospy.ROSInterruptException:
        pass