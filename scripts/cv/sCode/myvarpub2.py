#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import imutils
import numpy as np
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D

from Tkinter import *
count = 0
def publish0(data):
    pint = int(data)
    int_pub = rospy.Publisher("/myint0", Int32, queue_size=2)
    int_pub.publish(pint)

def publish1(data):
    pint = int(data)
    int_pub = rospy.Publisher("/myint1", Int32, queue_size=2)
    int_pub.publish(pint)

def main(args):
    #args: [[bottom value, top value, scale],...]
    #eg rosrun pe myvarpub2 0,1000 0,100

    rospy.init_node('myintpublisher', anonymous=True)
    global count

    print("running...")

    master = Tk()
    n = len(args)-1
    for i in range(1,len(args)):
        pars = (args[i].split(','))
        bot = str(pars[0])
        top = str(pars[1])
        if i == 1:
            w0 = Scale(master, from_=bot, to=top,orient='horizontal',command=publish0, width=60, length=400)
            w0.pack()
        elif i == 2:
            w1 = Scale(master, from_=bot, to=top,orient='horizontal',command=publish1, width=60, length=400)
            w1.pack()
    mainloop()


if __name__ == '__main__':
    main(sys.argv)
