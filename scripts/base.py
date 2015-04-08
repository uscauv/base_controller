#!/usr/bin/env python
# license removed for brevity

import serial

import rospy
from geometry_msgs.msg import Twist, Vector3


class BaseController():
    def __init__(self):
        self.port = rospy.get_param("/base_controller/port", 0)

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.spin()

    def callback(self, data):
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
        ser = Serial(port, 9600, timeout=1)
        ser.write("1 " + mt1)
        ser.write("2 " + mt2)
        ser.write("3 " + mt3)
        ser.write("4 " + mt4)
        ser.write("5 " + mt5)
        ser.write("6 " + mt6)


def listener():
    BaseController()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass