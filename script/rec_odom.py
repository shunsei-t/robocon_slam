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

import tf

class serial_:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=3)
        self.rec_flag = False
        self.rec_count = 0
        self.max_rec_count = 13
        self.idx = self.max_rec_count + 1
        self.rec_data = [0.0]*13
        self.pre_data = []
        self.odom = [0.0]*3

    def read_data(self):
        buffer = self.ser.read(self.max_rec_count)
        tmp = np.frombuffer(buffer, dtype=np.uint8, count=-1)
        tmp = tmp.tolist()
        if 255 in tmp:
            self.idx = tmp.index(255)
            if self.idx != self.max_rec_count + 1:
                now_tmp = tmp[self.idx:]
                self.rec_data = bytearray(now_tmp + self.pre_data)
                # print(now_tmp)
                # print(self.pre_data)
                # print("rec")
                # print(self.rec_data)
                self.convert_data()

                self.pre_data = tmp[:self.idx]
            elif self.idx == 0:
                self.rec_data = tmp
                self.convert_data()
        else:
            self.idx = self.max_rec_count + 1

    def convert_data(self):
        print(self.rec_data)
        self.odom[0] = self.rec_data[1]<<24 | self.rec_data[2]<<16 | self.rec_data[3]<<8 | self.rec_data[4]
        #self.odom[1] = self.rec_data[5]<<24 | self.rec_data[6]<<16 | self.rec_data[7]<<8 | self.rec_data[8]
        #self.odom[2] = self.rec_data[9]<<24 | self.rec_data[10]<<16 | self.rec_data[11]<<8 | self.rec_data[12]
        print(self.odom)
            
if __name__ == '__main__':
    try:
        rospy.init_node('rec_odom', anonymous=True)
        r = rospy.Rate(10)
        myserial = serial_()

        while not rospy.is_shutdown():
            myserial.read_data()
            r.sleep()


    except rospy.ROSInterruptException:
        ser.close()
        pass