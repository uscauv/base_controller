#!/usr/bin/env python
# license removed for brevity

import numpy as np
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

        print(str(mt1, mt2, mt3, mt4, mt5, mt6))

        # start byte = 255, 0 = full reverse, 100 = stopped, 200 = full forward
        def format_value(val):
            # first [0, 2], then 0 to 200
            return (val + 1) * 200

        # if any value above 1, normalize array so that that value becomes 1 and all others scale proportionally
        def normalize(arr):
            if np.max(np.abs(arr)) > 1:
                return arr / np.max(np.abs(arr))
            return arr

        message = bytearray([format_value(x) for x in normalize([mt1, mt2, mt3, mt4, mt5, mt6])])
        self.serial.write(message)
        self.serial.flush();


def base_controller():
    BaseController()


if __name__ == '__main__':
    try:
        base_controller()
    except rospy.ROSInterruptException:
        pass
