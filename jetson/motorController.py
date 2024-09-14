#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist

class motorController():
    def __init__(self):
        self.rate = rospy.Rate(100)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
        self.GPIO = GPIO
        self.GPIO.setmode(GPIO.BCM)
        self.motor1_EA = 18
        self.motor1_IA = 17
        self.motor2_EA = 16
        self.motor2_IA = 19

        self.GPIO.setup(self.motor1_EA, self.GPIO.OUT)
        self.GPIO.setup(self.motor1_IA, self.GPIO.OUT)
        self.GPIO.setup(self.motor2_EA, self.GPIO.OUT)
        self.GPIO.setup(self.motor2_IA, self.GPIO.OUT)

        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.callback)

    def shutdownhook(self):
        self.ctrl_c = True

    def callback(self, msg):
        if msg.linear.x > 0:
            self.GPIO.output(self.motor1_EA, True)
            self.GPIO.output(self.motor1_IA, False)
            self.GPIO.output(self.motor2_EA, True)
            self.GPIO.output(self.motor2_IA, False)
            rospy.loginfo('forward')
        elif msg.linear.x < 0:
            self.GPIO.output(self.motor1_EA, False)
            self.GPIO.output(self.motor1_IA, True)
            self.GPIO.output(self.motor2_EA, False)
            self.GPIO.output(self.motor2_IA, True)
            rospy.loginfo('backward')
        elif msg.angular.z > 0:
            self.GPIO.output(self.motor1_EA, True)
            self.GPIO.output(self.motor1_IA, False)
            self.GPIO.output(self.motor2_EA, False)
            self.GPIO.output(self.motor2_IA, True)
            rospy.loginfo('left')
        elif msg.angular.z < 0:
            self.GPIO.output(self.motor1_EA, False)
            self.GPIO.output(self.motor1_IA, True)
            self.GPIO.output(self.motor2_EA, True)
            self.GPIO.output(self.motor2_IA, False)
            rospy.loginfo('right')
        else:
            self.GPIO.output(self.motor1_EA, False)
            self.GPIO.output(self.motor1_IA, False)
            self.GPIO.output(self.motor2_EA, False)
            self.GPIO.output(self.motor2_IA, False)
            rospy.loginfo('stop')

    def start(self):
        while not rospy.is_shutdown() and not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("motorController", anonymous=True)
    motorControl = motorController()
    motorControl.start()