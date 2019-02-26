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
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)
    self.image_pub2 = rospy.Publisher("/myresult2", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)



  def callback(self,data):
    warpSuccess = False
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space
    lower_blue = np.array([90,80,50]) # All: 20,0,0 Blac/white: 0,0,250
    upper_blue = np.array([110,255,255]) # All: 300,300,300 Black/white: 255,5,255

    # define kerner for smoothing
    kernel = np.ones((3, 3), np.uint8)

    # Threshold the HSV image to get only the pixels in ranage
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # morph it
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

    # find contours usinig cv2 findContuors
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Define largest detected sign
    largestArea = 0
    largestCenter = None
    largestRadius = None
    largestRectArea = 0
    largestRect = None
    if len(contours) > 0:
        print("YAAAAS!! I CAN SEE A BLUE(!!!) SIGN")
        for cnt in contours:
            # Find enclosing circles and add to publish
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(res,center,radius,(0,255,0),2)
            cv2.putText(res, 'This is a BLUE sign', (center[0]-radius, center[1]+2*radius), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

            # Calculate area to determine the largest sign
            area = math.pi*radius*radius
            if area > largestArea:
                largestArea = area
                largestCenter = center
                largestRadius = radius


            # Find enclosing rectangle
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # count euclidian distance for each side of the rectangle
            sideOne = np.linalg.norm(box[0]-box[1])
            sideTwo = np.linalg.norm(box[0]-box[3])
            # count area of the rectangle
            areaRect = sideOne*sideTwo

            if areaRect > largestRectArea:
                largestRectArea = area
                largestRect = box


    elif len(contours) == 0:
        print("NO!!! I CAN'T SEE ANY BLUE SIGNS")

    if largestArea > 1000:
        #rospy.loginfo(box)
        # TODO: Section to detect specific sign. NOW: Detect which sign it is. Download simons packge or other way

        # Cut and warp interesting area
        warped = four_point_transform(mask, [largestRect][0])
        warpSuccess = True



        # detect which sign it is
        detectedTrafficSign = idSign(warped)
        #cv2.putText(res, detectedTrafficSign, (largestCenter[0]-largestRadius, largestCenter[1]+2*largestRadius), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)





    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8")) #"bgr8")
      if warpSuccess:
        self.image_pub2.publish(self.bridge.cv2_to_imgmsg(warped, "8UC1"))
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
