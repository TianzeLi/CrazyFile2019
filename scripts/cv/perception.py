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
# from transform import four_point_transform
from matplotlib import pyplot as plt
import imutils

from geometry_msgs.msg import PoseStamped

# Self-written functions
import idSign
import pose_tf

# Virtual bash
# cd ~/dd2419_ws/src/dd2419_perception_training/bags
# cd ~/dd2419_ws/src/pras_project/scripts/cv


class image_converter:

  def __init__(self):
    # publish the masked image with first contour extraction
    self.contour_image_pub = rospy.Publisher("/contourimage", Image, queue_size=2)

    # could pulsh something else you want for testing's sake
    # self.image_pub2 = rospy.Publisher("/homographyimage", Image, queue_size=2)
    # self.image_pub3 = rospy.Publisher("/result", Image, queue_size=2)

    self.location_sub = rospy.Subscriber("/cf1/pose", PoseStamped, self.pose_callback)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw/compressed", CompressedImage, self.image_callback)

  def pose_callback(self, data):
    pose = data
    # print(pose)

  def image_callback(self,data):
    # print('My eyes are open...')

    # fit compressed image into HSV space
    np_arr = np.fromstring(data.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # could blur the origin image here
    # cv_image = cv2.GaussianBlur(cv_image,(3,3),0)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


    # define range of the color we look for in the HSV space
    # OpenCV uses H: 0 - 180, S: 0 - 255, V: 0 - 255
    # lower_blue = np.array([110,70,90]) # All: 20,0,0 Black/white: 0,0,250, Blue: 90,80,50
    # upper_blue = np.array([135,255,200]) # All: 300,300,300 Black/white: 255,5,255, Blue: 110,255,255
    # lower_red1 = np.array([160,60,70]) # red color part 1 
    # upper_red1 = np.array([180,255,255])
    # lower_red2 = np.array([0,60,70]) # red color part 2
    # upper_red2 = np.array([5,255,255])
    # lower_ry3 = np.array([12,60,120]) # yellow color
    # upper_ry3 = np.array([23,255,255])

    lower_blue = np.array([100,102,40]) # All: 20,0,0 Black/white: 0,0,250, Blue: 90,80,50
    upper_blue = np.array([115,205,165]) # All: 300,300,300 Black/white: 255,5,255, Blue: 110,255,255
    lower_red1 = np.array([129,65,90]) # red color part 1 
    upper_red1 = np.array([180,255,205])
    lower_red2 = np.array([0,65,90]) # red color part 2
    upper_red2 = np.array([13,255,205])
    lower_ry3 = np.array([10,80,70]) # yellow color
    upper_ry3 = np.array([25,155,225])

    # combine the masks
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue) # BLUE!!
    mask_ry1 = cv2.inRange(hsv, lower_red1, upper_red1) # RED and YELLOW!
    mask_ry2 = cv2.inRange(hsv, lower_red2, upper_red2) # RED and YELLOW!
    mask_ry3 = cv2.inRange(hsv, lower_ry3, upper_ry3) # RED and YELLOW!
    mask_ry = cv2.bitwise_or(mask_ry1, mask_ry2 )
    mask_ry = cv2.bitwise_or(mask_ry, mask_ry3 )
    mask_total = cv2.bitwise_or(mask_b, mask_ry)
    
    # mask_total = cv2.GaussianBlur(mask_total,(3,3),0)

    # define kernels for morphology
    # could tune the kernel
    # kernel1 = np.ones((5, 5), np.uint8)
    # kernel1 = np.array([[1,1,1,1,1],
    #                     [1,1,1,1,1],
    #                     [1,1,1,1,1],
    #                     [1,1,1,1,1],
    #                     [1,1,1,1,1]], np.uint8)
    kernel1 = np.ones((3, 3), np.uint8)
    kernel2 = np.ones((4, 4), np.uint8)

    # # morph it, could tune the parameter "iterations"
    mask_total = cv2.dilate(mask_total, kernel1, iterations = 6)
    # mask_total = cv2.erode(mask_total,kernel2,iterations = 2)
    mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel2)
    # mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel2)

    # mask_total = cv2.erode(mask_total,kernel1,iterations = 1)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, mask= mask_total)
    # try:
    #     self.image_pub3.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
    # except CvBridgeError as e:
    #         print(e)
    # try:
    #     self.contour_image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8")) #"bgr8")
    # except CvBridgeError as e:
    #     print(e)    
            
    # # could set a thresold
    # imgray = cv2.cvtColor(masks[which_mask].copy(),cv2.COLOR_BGR2GRAY)
    # ret,thresh = cv2.threshold(imgray,127,255,0)

    # find contours usinig cv2 findContuors
    # Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object
    edges = cv2.Canny(res,120,360, L2gradient=True)
    contours = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # contours = cv2.findContours(mask_total.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # draw the contours
    contour_img = cv2.drawContours(hsv, contours, -1, (0,255,0), 2)

    # contour area restriction
    large_area = 2500
    small_area = 400

    boxes = []
    tmp_locs = []
    rects = [] 

    if len(contours) > 0:

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < small_area:
                # print('Area too small')
                continue
            if area > large_area:
                # print('Area too large')
                continue

            # make sure the contour has the right shape by checking the arc^2/area ratio, 
            # which value for circle, square or triangle should be less than 30, 22 actually.
            perimeter = cv2.arcLength(cnt,True)
            if perimeter*perimeter/area > 30:
                continue
                
            # Find enclosing rectangle
            rect = cv2.minAreaRect(cnt)

            loc_tmp = rect[0]
            reduc = 0
            if len(tmp_locs) > 1:
                for n_box in range(len(tmp_locs)):
                    if abs(tmp_locs[n_box][0] - loc_tmp[0]) < 10 \
                    or abs(tmp_locs[n_box][1] - loc_tmp[1]) < 10:
                        reduc = 1
                        continue
            if reduc == 1:
                continue
            else:
                tmp_locs.append(loc_tmp)

            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # check the angle of the rectangle
            angle = rect[2]
            if abs(angle) > 45:
                continue

            # check width-height ratio
            if rect[1][1] > 1.8*rect[1][0]:
                continue
            if rect[1][0] > 1.8*rect[1][1]:
                continue
            # print("Got a contour!")
            boxes.append([box])
            rects.append(rect)

    if len(boxes) > 0:
        # print("YAAAAS!! I CAN SEE A SIGN")
        for idex in range(len(boxes)):
            # sub_image = img[y:y+h, x:x+w]
            margin = 2
            sub_image = cv_image[int(rects[idex][0][1] - 0.5*rects[idex][1][1] - margin):\
                        int(rects[idex][0][1] + 0.5*rects[idex][1][1] + margin), \
                        int(rects[idex][0][0] - 0.5*rects[idex][1][0] - margin):\
                        int(rects[idex][0][0] + 0.5*rects[idex][1][0] + margin)]

            # get the number of the sign, 0 means not a sign
            sign_no = idSign.idSign(sub_image)

            # if sign_no != 0:
            #     u = recs[idex][0][0]
            #     v = recs[idex][0][1]
            #     sign_pose = pose_tf.pose_tf(pose, u, v, sign_no)
            #     print(sign_pose)

        # draw all the passed contours
        for idex in range(len(boxes)):
            selected_contour = cv2.drawContours(res,boxes[idex],0,(0,0,255),2)

        try:
            # self.contour_image_pub.publish(self.bridge.cv2_to_imgmsg(contour_img, "bgr8")) #"bgr8")
            self.contour_image_pub.publish(self.bridge.cv2_to_imgmsg(selected_contour, "bgr8")) #"bgr8")

        except CvBridgeError as e:
            print(e)

    #     try:
    #         # self.image_pub2.publish(self.bridge.cv2_to_imgmsg(res_red, "bgr8"))
    #         self.image_pub3.publish(self.bridge.cv2_to_imgmsg(result_pic, "bgr8"))
    #     except CvBridgeError as e:
    #         print(e)
    # else:
    #     print("No desired contour available")
    #     try:
    #         self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8")) #"bgr8")
    #         # self.image_pub2.publish(self.bridge.cv2_to_imgmsg(res_red, "bgr8"))
    #         # self.image_pub3.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
    #     except CvBridgeError as e:
    #         print(e)


def main(args):
  rospy.init_node('perception', anonymous=True)

  ic = image_converter()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
