#!/usr/bin/env python
# license removed for brevity

import serial

import rospy
from geometry_msgs.msg import Twist, Vector3


class BaseController():
    def __init__(self):
        self.port = rospy.get_param("/base_controller/port", 0)

        rospy.init_node('base_controller', anonymous=True)
        rospy.Subscriber("/cmd_vel_raw", Twist, self.callback)
        rospy.spin()

    def callback(self, data):
    	mutliplier = 400;
        linx = data.linear.x
        liny = data.linear.y
        linz = data.linear.z
        angx = data.angular.x
        angy = data.angular.y
        angz = data.angular.z

        mt1 = linx + angz  # x1 motor
        mt2 = linx - angz  # x2 motor
        mt3 = linz + angx  # z1 motor
        mt4 = linz - angx  # z2 motor
        mt5 = liny  # y1 motor
        mt6 = liny  # y2 motor
        print(mt1, mt2, mt3, mt4, mt5, mt6)
        ser = serial.Serial(port, 9600, timeout=1)
        '''
        ser.write("1 " + mt1*mutliplier)
        ser.write("2 " + mt2*mutliplier)
        ser.write("3 " + mt3*mutliplier)
        ser.write("4 " + mt4*mutliplier)
        ser.write("5 " + mt5*mutliplier)
        ser.write("6 " + mt6*mutliplier)
        '''
        ser.write("hello");
        ser.close();
        #s = ser.read(2)
        #print(s)



def base_controller():
    BaseController()


if __name__ == '__main__':
    try:
        base_controller()
    except rospy.ROSInterruptException:
        pass