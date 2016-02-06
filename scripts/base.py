#!/usr/bin/env python
# license removed for brevity

import serial
import time
import rospy
from geometry_msgs.msg import Twist, Vector3


class BaseController():
    def __init__(self):
        port = rospy.get_param("/base_controller/port", '/dev/seabee/arduino')
        self.serial = serial.Serial(port, 9600, timeout=1)

        rospy.init_node('base_controller', anonymous=True)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.callback)
        rospy.spin()

    def callback(self, data):
        multiplier = 200;
        linx = data.linear.x
        liny = data.linear.y
        linz = data.linear.z
        angx = data.angular.x
        angy = data.angular.y
        angz = data.angular.z

        mt1 = linx + angz  # x1 motor
        mt2 = -(linx - angz)  # x2 motor
        mt3 = linz + angx  # z1 motor
        mt4 = linz - angx  # z2 motor
        mt5 = liny  # y1 motor
        mt6 = -liny  # y2 motor
        #print(str(mt1, mt2, mt3, mt4, mt5, mt6)
        self.serial.write(b"1 " + str(mt1*multiplier) + "\r\n")
        self.serial.write(b"2 " + str(mt2*multiplier) + "\r\n")
        self.serial.write(b"3 " + str(mt3*multiplier) + "\r\n")
        self.serial.write(b"4 " + str(mt4*multiplier) + "\r\n")
        self.serial.write(b"5 " + str(mt5*multiplier) + "\r\n")
        self.serial.write(b"6 " + str(mt6*multiplier) + "\r\n")
        print("6 "+ str(mt6*multiplier));
        #self.serial.write("hello");
        self.serial.flush();
        #s = self.serial.read(2)
        #print(s)



def base_controller():
    BaseController()


if __name__ == '__main__':
    try:
        base_controller()
    except rospy.ROSInterruptException:
        pass
