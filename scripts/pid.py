#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class PidController():
    def __init__(self, p, i, d):
        self.setpoint = 0
        self.integral = 0
        self.last_error = 0

        self.p = p
        self.i = i
        self.d = d

    def set_pid(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

        self.integral = 0
        self.last_error = 0

    def update(self, val):
        error = self.setpoint - val
        self.integral += error

        # we don't divide by time here because we just assume that
        # the time in between updates is relatively constant, if this
        # is not the case, this code needs to be improved
        derivative = self.last_error - error

        return self.p * error + self.i * self.integral + self.d * derivative


class PidControllerNode():
    def __init__(self):
        self.pid_ang_x = PidController(1, 0, 0)
        self.pid_ang_y = PidController(1, 0, 0)
        self.pid_ang_z = PidController(1, 0, 0)

        # this is used to store the last velocity command
        self.command = None

        rospy.init_node('base_pid', anonymous=True)
        # listen to velocity command requests
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        # listen to IMU for feedback
        rospy.Subscriber("/nav_filtered_signals/filter_stack", Imu, self.imu_callback)
        # send commands to base controller
        self.pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=10)
        rospy.spin()

    def cmd_callback(self, data):
        print("Got command")
        self.command = data
        self.pid_ang_x.set_setpoint(0)
        self.pid_ang_y.set_setpoint(0)
        self.pid_ang_z.set_setpoint(0)

    def imu_callback(self, data):
        self.command = Twist()
        # we only modify angular values because this node is simply responsible for stabilizing the sub
        # making sure that the linear velocities are also accurate is outside the scope of this node
        self.command.angular.x = self.pid_ang_x.update(data.orientation.x)
        self.command.angular.y = self.pid_ang_y.update(data.orientation.y)
        self.command.angular.z = self.pid_ang_z.update(data.orientation.z)

        print("Got IMU update, publishing raw command")

        self.pub.publish(self.command)


def pid_controller():
    PidControllerNode()


if __name__ == '__main__':
    try:
        pid_controller()
    except rospy.ROSInterruptException:
        pass
