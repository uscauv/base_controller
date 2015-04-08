#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist, Vector3


def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        message = Twist()
        message.linear = Vector3(0.4, 0.5, 0.6)
        message.angular = Vector3(0.1, 0.2, 0.3)
        pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass