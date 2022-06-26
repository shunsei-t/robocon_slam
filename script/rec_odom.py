#!/usr/bin/env python
# coding: UTF-8

from __future__ import print_function

import serial
import rospy
import roslib
import numpy as np
import datetime
import struct

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf

class RecOdom:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=3)
        self.rec_flag = False
        self.rec_count = 0
        self.max_rec_count = 13
        self.idx = self.max_rec_count + 1
        self.pre_data = []
        self.odom_speed = [0.0]*3
        self.initial_odom = [0.0]*3
        self.br = tf.TransformBroadcaster()

    def read_data(self):
        buffer = self.ser.read(self.max_rec_count)
        tmp = struct.unpack_from("B"*13, buffer)
        tmp = list(tmp)
        #print(tmp)
        if 255 in tmp:
            self.idx = tmp.index(255)
            #print(idx)
            if self.idx != self.max_rec_count + 1:
                now_tmp = tmp[self.idx:]
                rec_data = now_tmp + self.pre_data
                #print(now_tmp)
                #print(self.pre_data)
                #print("rec")
                #print(rec_data)
                if len(rec_data) == 13:
                    self.convert_data(rec_data)

                self.pre_data = tmp[:self.idx]
            elif self.idx == 0 & len(tmp) == 13:
                self.convert_data(tmp)
        else:
            self.idx = self.max_rec_count + 1

    def convert_data(self, rec_data):
        #print(rec_data[1])
        a = struct.pack('I', rec_data[4]<<24 | rec_data[3]<<16 | rec_data[2]<<8 | rec_data[1])
        b = struct.pack('I', rec_data[8]<<24 | rec_data[7]<<16 | rec_data[6]<<8 | rec_data[5])
        c = struct.pack('I', rec_data[12]<<24 | rec_data[11]<<16 | rec_data[10]<<8 | rec_data[9])
        #struct.unpack('f', a)だと(n,)という形で出てくるので、listにして一番目を取り出し
        self.odom_speed[0] = list(struct.unpack('f', a))[0]
        self.odom_speed[1] = list(struct.unpack('f', b))[0]
        self.odom_speed[2] = list(struct.unpack('f', c))[0]
        print(self.odom_speed)

    def initialize_tf(self, data):
        q = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,
                                                    data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z,
                                                    data.pose.pose.orientation.w))
        self.initial_odom = [data.pose.pose.position.x, data.pose.pose.position.y, q[2]]
        print("initialize", self.initial_odom)

            
if __name__ == '__main__':
    try:
        rospy.init_node('rec_odom', anonymous=True)
        Hz = 10.0
        dt = 1.0/Hz
        r = rospy.Rate(Hz)
        myodom = RecOdom()

        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, myodom.initialize_tf)

        try:
            while not rospy.is_shutdown():
                myodom.read_data()

                next_odom = [0.0]*3
                next_odom[0] = myodom.initial_odom[0] + dt*myodom.odom_speed[0]
                next_odom[1] = myodom.initial_odom[1] + dt*myodom.odom_speed[1]
                next_odom[2] = myodom.initial_odom[2] + dt*myodom.odom_speed[2]
                #print(type(myodom.initial_odom[0]))

                myodom.br.sendTransform((next_odom[0], next_odom[1], 0),
                                tf.transformations.quaternion_from_euler(0, 0, next_odom[2]),
                                rospy.Time.now(),
                                "odom",
                                "map")

                r.sleep()
        except KeyboardInterrupt:
            myodom.ser.close()

    except rospy.ROSInterruptException:
        pass
