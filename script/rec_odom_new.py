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
        self.pre_data = []
        self.odom = [0.0]*3

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
        self.odom[0] = list(struct.unpack('f', a))[0]
        self.odom[1] = list(struct.unpack('f', b))[0]
        self.odom[2] = list(struct.unpack('f', c))[0]
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
