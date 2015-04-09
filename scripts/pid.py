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
        self.pid_ang_x = PidController(0.001, 0, 0)
        self.pid_ang_y = PidController(0.001, 0, 0)
        self.pid_ang_z = PidController(0.001, 0, 0)

        # this is used to store the last velocity command
        self.command = None

        rospy.init_node('base_pid', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        rospy.Subscriber("nav_filtered_signals/filter_stack", Imu, self.imu_callback)
        self.pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=10)
        rospy.spin()

    def cmd_callback(self, data):
        self.command = data
        self.pid_ang_x.set_setpoint(data.angular.x)
        self.pid_ang_y.set_setpoint(data.angular.y)
        self.pid_ang_z.set_setpoint(data.angular.z)

    def imu_callback(self, data):
        # we only modify angular values because this node is simply responsible for stabilizing the sub
        # making sure that the linear velocities are also accurate is outside the scope of this node
        self.command.angular.x = self.pid_ang_x.update(data.angular_velocity.x)
        self.command.angular.y = self.pid_ang_y.update(data.angular_velocity.y)
        self.command.angular.z = self.pid_ang_z.update(data.angular_velocity.z)

        self.pub.publish(self.command)


def pid_controller():
    PidControllerNode()


if __name__ == '__main__':
    try:
        pid_controller()
    except rospy.ROSInterruptException:
        pass