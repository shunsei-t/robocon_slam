#!/usr/bin/env python
# coding: UTF-8

from __future__ import print_function
import imp

import rospy
import roslib
import numpy as np
from std_msgs.msg import Bool

class Pursuit:
    def __init__(self):
        self.auto_flag = False
        loot_dir = roslib.packages.get_pkg_dir("robocon_slam")
        self.path = np.loadtxt(loot_dir+"/path/path.csv", delimiter=',')

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