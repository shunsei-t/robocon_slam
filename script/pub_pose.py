#!/usr/bin/env python
# coding: UTF-8

from __future__ import print_function

import rospy
import serial
import struct
from geometry_msgs.msg import Twist

class pub_pose:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=3)
        self.vel = [0.0]*3
        self.write_data = ['a']*13

    def vel_CB(self, data):
        self.vel[0] = data.linear.x
        self.vel[1] = data.linear.y
        self.vel[3] = data.angular.z

    def write(self):
        data = [0.01, 0.02, -0.01]
        tmp = struct.pack('fff', data[0], data[1], data[2])
        tmp = struct.unpack('B'*12, tmp)
        #print('send data 255+ ', end = '')
        #print(tmp)
        self.ser.write(255)
        for d in tmp:
            self.ser.write(d)


if __name__ == '__main__':
    try:
        rospy.init_node('pub_pose', anonymous=True)
        Hz = 10
        r = rospy.Rate(Hz)

        mypose = pub_pose()

        rospy.Subscriber("cmd_vel", Twist, mypose.vel_CB)

        try:
            while not rospy.is_shutdown():
                mypose.write()

                r.sleep()
        except KeyboardInterrupt:
            mypose.ser.close()
    except rospy.ROSInterruptException:
        pass