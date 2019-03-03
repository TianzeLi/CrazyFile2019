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

class extractsigns:
    def __init__(self):
        self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)
        self.image_pub2 = rospy.Publisher("/myresult2", Image, queue_size=2)
        self.image_pub3 = rospy.Publisher("/myresult3", Image, queue_size=2)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.segmentColors)

    def segmentColors(self, data):
        print('My eyes are open...')
        # Params
        areaThresholdDetection = 1500

        # Convert the image from OpenCV to ROS format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define HSV ranges
        lowerHsv, upperHsv = self.hsvRanges()
        colors = np.array(['Blue', 'Red', 'Yellow'])

        # define kerner for smoothing
        kernel = np.ones((3, 3), np.uint8)

        # 
        for i in range(0, 3):
            lower = lowerHsv[i]
            upper = upperHsv[i]

            # Threshold the HSV image to get only the pixels in ranage
            mask = cv2.inRange(hsv, lower, upper)

            # morph it
            #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(cv_image, cv_image, mask= mask)

            # find contours usinig cv2 findContuors
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            # If we find any contours then we try to find shapes
            if len(contours) > 0:
                # calculate some sort of area and threshold it
                # For the triggered color create some sort of feature vector
                for cnt in contours:
                    # Find enclosing circles and add to publish
                    (x,y),radius = cv2.minEnclosingCircle(cnt)
                    center = (int(x),int(y))
                    radius = int(radius)
                    area = math.pi*radius*radius
                    # ONLY for large enough areas
                    if area > areaThresholdDetection:
                        cv2.circle(res,center,radius,(0,255,0),2)
                        cv2.putText(res, 'This is a sign', (center[0]-radius, center[1]+2*radius), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                        #print("YAAAAS!! I CAN SEE A SIGN")

                        self.detectShapes(mask, res, contours)
                        print(colors[i])

    def detectShapes(self, mask, res, contours):
        print('I can see...')




    def hsvRanges(self):
        # Define HSV ranges
        # Blue
        lower_blue = np.array([105,80,50]) 
        upper_blue = np.array([120,255,255]) 
        # Red
        lower_red = np.array([0,70,70])
        upper_red = np.array([15,255,255])
        # Yellow
        lower_yellow = np.array([25,70,70])
        upper_yellow = np.array([50,255,255])

        lower = np.array([lower_blue, lower_red, lower_yellow])
        upper = np.array([upper_blue, upper_red, upper_yellow])

        return lower, upper



def main(args):
    rospy.init_node('colorseg', anonymous=True)

    es = extractsigns()

    print("running...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
