#!/usr/bin/env python

from sensor_msgs.msg import Joy
import sys, rospy
from geometry_msgs.msg import Twist

class ps4Controller():
    def __init__(self):
        self.cmdPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.joySub = rospy.Subscriber('/joy', Joy, self.joyCallback)
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.ctrl_c = True


    def joyCallback(self, data):
        self.twist.linear.x = data.axes[1]
        self.twist.angular.z = data.axes[2]
        self.cmdPub.publish(self.twist)

    def main(self):
        while not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ps4Controller', anonymous=True)
    ps4 = ps4Controller()
    try:
        ps4.main()
    except rospy.ROSInterruptException:
        pass