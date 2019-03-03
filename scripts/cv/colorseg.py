#!/usr/bin/env python
from __future__ import print_function

import math
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from transform import four_point_transform


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/cameraimage_blue", Image, queue_size=2)
    self.image_pub2 = rospy.Publisher("/cameraimage_red", Image, queue_size=2)
    self.image_pub3 = rospy.Publisher("/cameraimage_yellow", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)



  def callback(self,data):
    print('My eyes are open...')
    warpSuccess = False
    areaThresholdCircles = 1500
    areaThresholdDetection = 1500

    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space
    lower_blue = np.array([105,80,50]) # All: 20,0,0 Black/white: 0,0,250, Blue: 90,80,50
    upper_blue = np.array([120,255,255]) # All: 300,300,300 Black/white: 255,5,255, Blue: 110,255,255

    lower_red = np.array([0,70,70])
    upper_red = np.array([13,255,255])

    lower_yellow = np.array([15,70,70])
    upper_yellow = np.array([30,255,255])



    # define kerner for smoothing
    kernel = np.ones((3, 3), np.uint8)

    # Threshold the HSV image to get only the pixels in ranage
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue) # BLUE!!
    mask_red = cv2.inRange(hsv, lower_red, upper_red) # RED!!
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow) # YELLOW!!

    # morph it
    mask_blue= cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    
    mask_red= cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    
    mask_yellow= cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

    # Bitwise-AND mask and original image
    res_blue = cv2.bitwise_and(cv_image, cv_image, mask= mask_blue)
    res_red = cv2.bitwise_and(cv_image, cv_image, mask= mask_red)
    res_yellow = cv2.bitwise_and(cv_image, cv_image, mask= mask_yellow)

    # find contours usinig cv2 findContuors
    #contours_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    #contours_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    #contours_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]


    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(res_blue, "bgr8")) #"bgr8")
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(res_red, "bgr8"))
      self.image_pub3.publish(self.bridge.cv2_to_imgmsg(res_yellow, "bgr8"))
    except CvBridgeError as e:
      print(e)


def idSign(image):
    return 'This is a sign'



def main(args):
  rospy.init_node('colorseg', anonymous=True)

  ic = image_converter()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
