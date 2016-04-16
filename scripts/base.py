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


        # start byte = 255, 0 = full reverse, 100 = stopped, 200 = full forward
    def format_value(self, val):
        # first [0, 2], then 0 to 200
        val = int((val + 1.0)/2.0 * 200 + 0)
        return min(255, max(val, 0))

        # if any value above 1, normalize array so that that value becomes 1 and all others scale proportionally
    def normalize(self, arr):
        if np.max(np.abs(arr)) > 1:
            return arr / np.max(np.abs(arr))
        return arr


    def callback(self, data):
        linx = float(data.linear.x)
        liny = float(data.linear.y)
        linz = float(data.linear.z)
        angx = float(data.angular.x)
        angy = float(data.angular.y)
        angz = float(data.angular.z)

	#
	mt1 = linz-angy #non-connector end, facing up/down
	mt2 = -(linz+angy) #connector end, facing up/down
	mt3 = linx + angy #top motor, by killswitch
	mt4 = liny #looking at the connector end, left side, facing front/back
	mt5 = (linx-angx) #bottom motor
	mt6 =  -liny #right side looking at the connector end, facing front/back
	
#        mt1 = 0
#        mt2 = .7
       # mt3 = 0
#        mt4 = 0
#        mt5 = 0
#        mt6 = 0

        print(str(mt1), str(mt2), str(mt3), str(mt4), str(mt5), str(mt6))

        message_bytes = [self.format_value(x) for x in self.normalize([mt1, mt2, mt3, mt4, mt5, mt6])]
	message_bytes.insert(0, 255)
	print(self.normalize([mt1,mt2,mt3,mt4,mt5,mt6]))
	print(message_bytes)
        message = bytearray(message_bytes)
        self.serial.write(message)
        self.serial.flush();

def base_controller():
    BaseController()


if __name__ == '__main__':
    try:
        base_controller()
    except rospy.ROSInterruptException:
        pass
