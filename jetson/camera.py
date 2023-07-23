#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        self.rate = rospy.Rate(100)
        self.ctrl_c = False
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.ctrl_c = True

    def start(self):
        while not rospy.is_shutdown() and not self.ctrl_c:
            rospy.loginfo('publishing image')
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image))
            self.rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()