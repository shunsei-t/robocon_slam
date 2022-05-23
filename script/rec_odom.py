#!/usr/bin/env python
# coding: UTF-8

from __future__ import print_function

import serial
import rospy
import roslib
import numpy as np
import datetime

from std_msgs.msg import Float32

import tf

class serial_:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=3)
        self.rec_flag = False
        self.rec_data = [0]*13
        self.rec_count = 0

    def read_data(self):
        line = self.ser.read(13)
        tmp = np.frombuffer(line, dtype=np.uint8, count=-1)
        #tmp = int(tmp)
        print(tmp.astype(np.uint8))

        # if tmp[0] == 255:
        #     if self.rec_count == 0 or self.rec_count == 7:
        #         self.rec_flag = True
        #     else:
        #         self.rec_flag = False

        # if self.rec_flag == True:
        #     self.rec_data[self.rec_count] = tmp
        #     self.rec_count += 1
            
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