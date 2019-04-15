#!/usr/bin/env python
from __future__ import print_function

import math
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from transform import four_point_transform
from matplotlib import pyplot as plt


# rosrun image_transport republish compressed in:=/cf1/camera/image_raw/compressed raw out:=/cf1/camera/image_raw

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/cameraimage", Image, queue_size=2)
    self.image_pub2 = rospy.Publisher("/homographyimage", Image, queue_size=2)
    self.image_pub3 = rospy.Publisher("/colorimage", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw/compressed", CompressedImage, self.callback)



  def callback(self,data):
    print('My eyes are open...')
    warpSuccess = False
    areaThresholdCircles = 1500
    areaThresholdDetection = 1500

    # Convert the image from ROS to OpenCV format
    # try:
    #   cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    # except CvBridgeError as e:
    #   print(e)

    np_arr = np.fromstring(data.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space
    lower_blue = np.array([105,80,50]) # All: 20,0,0 Black/white: 0,0,250, Blue: 90,80,50
    upper_blue = np.array([120,255,255]) # All: 300,300,300 Black/white: 255,5,255, Blue: 110,255,255

    lower_red = np.array([0,70,70])
    upper_red = np.array([13,255,255])

    lower_yellow = np.array([15,70,70])
    upper_yellow = np.array([25,255,255])



    # define kerner for smoothing
    kernel = np.ones((3, 3), np.uint8)

    # Threshold the HSV image to get only the pixels in ranage
    mask = cv2.inRange(hsv, lower_blue, upper_blue) # BLUE!!
    #mask = cv2.inRange(hsv, lower_red, upper_red) # RED!!
    #mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # YELLOW!!

    # morph it
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, mask= mask)
    
    # # could set a thresold
    # imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    # ret,thresh = cv2.threshold(imgray,127,255,0)

    # find contours usinig cv2 findContuors
    # Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # # draw the contours
    # img = cv2.drawContours(img, contours, -1, (0,255,0), 3)

    # # contour arc length
    # perimeter = cv2.arcLength(cnt,True)

    # # contour area
    # area = cv2.contourArea(cnt)

    # Define largest detected sign
    largestArea = 0
    largestCenter = None
    largestRadius = None
    largestRectArea = 0
    largestRect = None
    if len(contours) > 0:
        
        for cnt in contours:
            # Find enclosing circles and add to publish
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            area = math.pi*radius*radius

            # Calculate area to determine the largest sign            
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

            # Save large enough areas
            if area > areaThresholdCircles:
              #cv2.circle(res,center,radius,(0,255,0),2)
              cv2.drawContours(res,[largestRect],0,(0,0,255),2)
              cv2.putText(res, 'This is a quite large area of color', \
                (center[0]-radius, center[1]+2*radius), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
              print("YAAAAS!! I CAN SEE A SIGN")
            else:
              print('But I cant see anything... :(')


    elif len(contours) == 0:
        print("NO!!! I CAN'T SEE ANY BLUE SIGNS")

    #print(largestArea)
    #print(largestRectArea)


    if largestArea > areaThresholdDetection:
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
      self.image_pub3.publish(self.bridge.cv2_to_imgmsg(hsv, "bgr8"))
      if warpSuccess:
        self.image_pub2.publish(self.bridge.cv2_to_imgmsg(warped, "8UC1"))
    except CvBridgeError as e:
      print(e)


def idSign(image):

    # #
    # cv2.matchShapes() 

    # cv2.matchTemplate()

    # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(results)


    img = image
    img2 = img.copy()
    template = cv2.imread('/home/tianze/dd2419_ws/src/dd2419_perception_training/signs/follow_left.png',0)
    w, h = template.shape[::-1]

    # All the 6 methods for comparison in a list
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
            'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

    for meth in methods:
        img = img2.copy()
        method = eval(meth)

        # Apply template Matching
        res = cv2.matchTemplate(img,template,method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)

        cv2.rectangle(img,top_left, bottom_right, 255, 2)

        plt.subplot(121),plt.imshow(res,cmap = 'gray')
        plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(img,cmap = 'gray')
        plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
        plt.suptitle(meth)

        plt.show()
    
    # return 'This is a sign'


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
