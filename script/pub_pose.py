from __future__ import print_function

import rospy
import serial
import struct
from geometry_msgs import Twist

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
        self.ser.write("a")
        

if __name__ == '__main__':
    try:
        rospy.init_node('pub_pose.py', anonymous=True)
        Hz = 20
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