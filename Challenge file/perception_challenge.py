#!/usr/bin/env python
from __future__ import print_function

import math
import roslib
import sys
import os, copy, math
import rospy
import tf2_ros
import cv2, imutils
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from transform import four_point_transform
from matplotlib import pyplot as plt
import csv

from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler

# Self-written functions
import idSign
import pose_tf

# Keras tensorflow
from keras.models import load_model, Sequential

# Import positioning
#from positioning import Positioning

modelFolder = '/home/robot/dd2419_ws/src/pras_project/Challenge file/'
singelModel = load_model(modelFolder + 'Single' + '8' + '.h5')
singelModel._make_predict_function()

#bridge = CvBridge()
deg2rad = math.pi/180

#TODO: 
# Create an object for the signPoseMatrix so that it saves every poistion and not
# only for the single image. Need to have its own object.

# Clustering and determine position of sign in world
# Positioning
#   - 
# Save all positions to temporary CSV file
# Postprocess with clustering (cv2.solvePnp)
# Later on if time:
#   - Improve post processeing by removing the background aka making it white
#       * Use black background
#       * Put contour
#       * Make background white

DICT_NAMES = {'airport' :   0,
                'dangerous_curve_left' :    1,
                'dangerous_curve_right' :   2,
                'follow_left' :     3,
                'follow_right' :    4,
                'junction' :    5,
                'no_bicycle' :  6,
                'no_heavy_truck':   7,
                'no_parking' :  8,
                'no_stopping_and_parking' :     9,
                'residential' :     10,
                'road_narrows_from_left' :  11,
                'road_narrows_from_right' :     12,
                'roundabout_warning' :     13,
                'stop' :    14
                }

class signDetector:
    def __init__(self, bridge):
        self.bridge = bridge
        #self.img = image
        #self.signPoseMatrix = []
        """
        with open('cluster_data.csv', 'w') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerows([["ID","x","y","z"]])
        csvFile.close()
        """
        self.list_loc_signs = []

        #self.mainDetector(image)
        #self.positioner = Positioning()
        #img_sub = rospy.Subscriber("/cf1/camera/image_raw", Image,imageCallback)
        self.img_sub = rospy.Subscriber("/cf1/camera/image_raw/decompressed", Image,self.imageCallback)
        self.pose_sub = rospy.Subscriber("/cf1/pose", PoseStamped, self.poseCallback)

        # publish the masked image with first contour extraction
        self.contour_image_pub = rospy.Publisher("/contourimage", Image, queue_size=2)

        self.counter_detected_signs = 0 # when we reach 500 signs seing I save them in a CSV file and empty the array to start again
        self.clusters_arr = np.empty((0,6)) # where I save the signs with its position. [Label,X,Y,Z,counter,index]
        # where counter is times I have seen the sign in a small area (number elements cluster), index is just to keep track of number clusters
        # and X,Y,Z are average of the signs of each cluster
        self.index_array = 0 # counter of number of clusters
    
    def initImage(self, image):
        self.img = image
        self.mainDetector(image)

    def mainDetector(self, img):
        # **PREPROCESS IMAGE***
        # copy image for reasons
        image = img.copy() 
        # filter image to improve edge detection
        imageBlurred = cv2.bilateralFilter(image, 5, 80, 80)
        # grayscale and normalise image
        imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # normalise image
        imageNormalised = imageGray.copy()
        cv2.normalize(imageGray, imageNormalised, 50, 230, cv2.NORM_MINMAX)
        # blurr image
        imageBlurred = cv2.GaussianBlur(imageNormalised, (5, 5), 0)

        # **DETECT EDGES AND CONTOURS**
        # canny edge detection
        edgesDetected = cv2.Canny(imageBlurred, 10, 3, 3)
        self.publishImage(edgesDetected, 'canny')
        # find contours
        edges = edgesDetected.copy()
        contours = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        #contours = imutils.grab_contours(contours) 
        # OBS DIFFERENCE: [-2] at end of findContours and no grab_contours
        #contour_img = cv2.drawContours(image, contours, -1, (0,255,0), 2)
        #publishImage(contour_img, 'contours')

        imageContours = image.copy()
        #positioner = Positioning()
        if len(contours) > 0:
            for cnt in contours:
                # Area restrictor
                area = cv2.contourArea(cnt)
                #print(area)
                if area <= 70 or area > 2500:
                    #print("No contour within limits")
                    continue

                # Convexity restrictor
                # make sure the contour has the right shape by checking the arc^2/area ratio, 
                # which value for circle, square or triangle should be less than 30, 22 actually.
                arcLen = cv2.arcLength(cnt, True)
                polyApprox = cv2.approxPolyDP(cnt, 0.1*arcLen, True)
                if cv2.isContourConvex(polyApprox) == False:
                    continue

                # Find enclosing rectangle
                rect = cv2.minAreaRect(cnt)

                # Warp image
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                imageWarped = self.warpImage(image, box)

                # Check color content of warped image
                colorContent = self.checkColors(imageWarped) #[Blue, Red, Yellow]
                maxColorContent = np.amax(colorContent)
                #print(maxColorContent)
                if maxColorContent < 0.05:
                    #print("NOT ENOGH COLORS FFS")
                    continue

                # Control shape of contour
                shape = self.determineContourShape(cnt)
                if shape == 'none':
                    continue
                elif shape == 'rectangle':
                    blueContent = colorContent[0]
                    if blueContent < 0.05:
                        continue
                elif shape == 'triangle':
                    yellowContent = colorContent[2]
                    if yellowContent < 0.05:
                        continue
                elif shape == 'circle':
                    pass

                # Determine position of contour
                x,y,w,h = cv2.boundingRect(cnt)
                center = (x,y)

                #Draw contour on image and classify using NN:
                self.publishImage(imageWarped, 'warp')
                classifiedSign = self.classifySign(imageWarped)

                cv2.drawContours(imageContours, [box], -1, (0, 255, 9), 2) # draw box
                cv2.putText(imageContours, classifiedSign, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)           
                
                self.positionSign(shape, box, classifiedSign)
                
        self.publishImage(imageContours, 'imageWithBoxes')


    
    def classifySign(self, image):
        #print('IM CLASSIFYING!!! YAYA')
        model = singelModel
        classNames = ['airport', 
                        'dangerous_curve_left',
                        'dangerous_curve_right',
                        'follow_left',
                        'follow_right',
                        'junction',
                        'no_bicycle',
                        'no_heavy_truck',
                        'no_parking',
                        'no_stopping_and_parking',
                        'residential',
                        'road_narrows_from_left',
                        'road_narrows_from_right',
                        'roundabout_warning',
                        'stop',
                        'z_crap']
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        imageExpand = np.expand_dims(np.array(imageRGB), 0)
        predict = model.predict(imageExpand)
        signPredictedPos = np.where(predict == np.amax(predict))[1][0]
        signPredicted = classNames[signPredictedPos]
        return signPredicted

        
    def determineContourShape(self, cnt):
        area = cv2.contourArea(cnt)
        arcLen = cv2.arcLength(cnt, True)
        # Determine min ecnlosing triangle
        val, triangle = cv2.minEnclosingTriangle(cnt)
        triangleArea = cv2.contourArea(triangle)
        # Determine min enclosing rectangle
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        rectangle = np.int0(box)
        rectangleArea = cv2.contourArea(rectangle)
        # Determine min enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        radius = int(radius)
        circleArea = radius*radius*math.pi

        
        triangleRatio = area/triangleArea
        rectangleRatio = area/rectangleArea
        circleRatio = area/circleArea
        errors = [np.sqrt(np.power(triangleRatio-1,2)), np.sqrt(np.power(rectangleRatio-1,2)), np.sqrt(np.power(circleRatio-1,2))]

        minError = np.amin(errors)
        minErrorPos = np.where(errors == minError)[0][0]
        signShapes = ['triangle', 'rectangle', 'circle']
        if minError > 0.2:
            shape = 'none'
        else:
            shape = signShapes[minErrorPos]

        return shape


    def checkColors(self, image):
        #print("Checking colors")
        width, height = image.shape[:2]
        area = width*height
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges
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
        mask_ry4 = cv2.bitwise_or(mask_ry1, mask_ry2 )
        mask_ry = cv2.bitwise_or(mask_ry4, mask_ry3 )
        mask_total = cv2.bitwise_or(mask_b, mask_ry)

        # Blue color content
        masked_blue = cv2.bitwise_and(image, image, mask= mask_b)
        gray_b = cv2.cvtColor(masked_blue, cv2.COLOR_BGR2GRAY)
        thresh_b = cv2.threshold(gray_b, 0, 255, cv2.THRESH_BINARY)[1]
        result_b = float(cv2.countNonZero(thresh_b))/area

        # Red color content
        masked_red = cv2.bitwise_and(image, image, mask= mask_ry4)
        gray_r = cv2.cvtColor(masked_red, cv2.COLOR_BGR2GRAY)
        thresh_r = cv2.threshold(gray_r, 0, 255, cv2.THRESH_BINARY)[1]
        result_r = float(cv2.countNonZero(thresh_r))/area

        # Yellow color content
        masked_yellow = cv2.bitwise_and(image, image, mask= mask_ry3)
        gray_y = cv2.cvtColor(masked_yellow, cv2.COLOR_BGR2GRAY)
        thresh_y = cv2.threshold(gray_y, 0, 255, cv2.THRESH_BINARY)[1]
        result_y = float(cv2.countNonZero(thresh_y))/area

        colorContent = [result_b, result_r, result_y]

        return colorContent

                
    def warpImage(self, image, box):
        warp = four_point_transform(image, [box][0])
        # Resize for NN classification
        classificationDim = (64, 64)
        imageWarped = cv2.resize(warp, classificationDim, interpolation = cv2.INTER_LANCZOS4)
        #print("warp")
        return imageWarped

    def positionSign(self, shape, box, sign):
        cameraMatrix = np.array(
                [[231.250001, 0.000000, 320.519378],
                [0.0000000, 231.06552, 240.631482],
                [0.0000000, 0.000000, 1.0000000]])
        distortionCoefficients = np.array([0.061687, -0.049761, -0.008166, 0.004284, 0.0])
            
        if shape == 'circle':
            signWitdh = 0.2
            signHeight = 0.2
        elif shape == 'triangle':
            signWitdh = 0.2
            signHeight = 0.18
        elif shape == 'rectangle':
            signWitdh = 0.2
            signHeight = 0.145

        objectPoints = np.array([
                                    (-signWitdh/2, signHeight/2, 0), # top left
                                    (signWitdh/2, signHeight/2, 0), # top right
                                    (signWitdh/2, -signHeight/2, 0), # lower right
                                    (-signWitdh/2, -signHeight/2, 0)  # lower left

                                ])

        #imagePoints = np.float32(box)
        
        imagePoints = np.zeros((4, 2), dtype="float32")
        # the top-left point will have the smallest sum, whereas
        # the bottom-right point will have the largest sum
        s = box.sum(axis=1)
        imagePoints[0] = box[np.argmin(s)]
        imagePoints[2] = box[np.argmax(s)]
    
        # now, compute the difference between the points, the
        # top-right point will have the smallest difference,
        # whereas the bottom-left will have the largest difference
        diff = np.diff(box, axis=1)
        imagePoints[1] = box[np.argmin(diff)]
        imagePoints[3] = box[np.argmax(diff)]

        """
        imagePoints = np.array([
                                (box[0][0]  , box[0][1]), # 1st point
                                (box[1][0]  , box[1][1]), # 2nd point
                                (box[2][0]  , box[2][1]), # 3rd point
                                (box[3][0]  , box[3][1])  # 4rth point
                            ], dtype = "double")
        """
        (success, rotation_vector, translation_vector) = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distortionCoefficients)

        if success and sign != 'z_crap':
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            t = PoseStamped()
            t.header.frame_id = 'cf1/camera'
            t.header.stamp = rospy.Time()
            

            t.pose.position.x = translation_vector[0][0]
            t.pose.position.y = translation_vector[1][0]
            t.pose.position.z = translation_vector[2][0]
            posit = t.pose.position
            #print([sign, posit.x, posit.y, posit.z])

            (t.pose.orientation.x, 
            t.pose.orientation.y, 
            t.pose.orientation.z, 
            t.pose.orientation.w)  = quaternion_from_euler(rotation_vector[0], rotation_vector[1], rotation_vector[2])
            
            #t.pose.orientation = np.zeros(4)
            trans = tf_buf.transform(t,'cf1/odom')
            # rosbag.duration(0.5)
            tOut = TransformStamped()
            tOut.header.frame_id = 'map'
            tOut.child_frame_id = 'cf1/' + sign
            tOut.transform.translation = trans.pose.position
            tOut.transform.rotation = trans.pose.orientation

            broadcaster.sendTransform(tOut)

            ## ----------------------Clustering--------------------------------

            ID_sign = DICT_NAMES[sign] # Codify label as Integer
            #print("11111111111 : " + str(ID_sign))
            
            # Filter to just get the elements of the same sign
            #print("BEGIN: " + str(self.clusters_arr))
            filtered_by_sign = self.clusters_arr[self.clusters_arr[:, 0] == ID_sign]
            

            #print(['self.clusters_arr[:, 0] = ',  self.clusters_arr[:, 0]])
            #print(['filtered_by_sign = ', filtered_by_sign])
            if filtered_by_sign.size != 0: # Check if that sign is in the cluster array or not, if no add it but if yes
            # check if the detecetd sign is close to the cluster of the same sign type detected before
                flag = 0
                for elements in filtered_by_sign:
                    # if distance less than 2 meters
                    if np.abs(posit.x - elements[1]) + np.abs(posit.y - elements[2]) + np.abs(posit.z - elements[3]) < 2:
                        av_x = (posit.x + elements[1]*elements[4])/float(elements[4]+1) # update average position cluster with new measurement
                        av_y = (posit.y + elements[2]*elements[4])/float(elements[4]+1)
                        av_z = (posit.z + elements[3]*elements[4])/float(elements[4]+1)
                        self.clusters_arr[self.clusters_arr[:,5] == elements[5], :] = [ID_sign, av_x, av_y, av_z, elements[4]+1, elements[5]]
                        flag = 1
                        
                        break
                if flag == 0: # no sign close to it, so create it
                    self.clusters_arr = np.append(
                        self.clusters_arr,
                        np.array([[ID_sign, posit.x, posit.y, posit.z, 1, self.index_array]]),
                        axis = 0)
                    self.index_array = self.index_array+1
            else:
                self.clusters_arr = np.append(
                    self.clusters_arr,
                    np.array([[ID_sign, posit.x, posit.y, posit.z, 1, self.index_array]]),
                    axis = 0)
                #print("SHAPE: " + str(self.clusters_arr))
                self.index_array = self.index_array+1



    def writeRowToCSV(fileName, sign, position):

        with open(fileName+'.csv', mode='w') as csvFile:
            csvWriter = csv.writer(csvFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            csvWriter.writerow([sign, position])

      

    def imageCallback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Initialse image in class and run mainDetector function
        self.mainDetector(image) 

        ## ------ Save every 500 detected signs on a CSV -----------------------------
        if self.counter_detected_signs == 500:
            #process
            self.clusters_arr = self.clusters_arr[self.clusters_arr[:, 4] > 13] # check has more than 13 elements
            
            #with open('cluster_data.csv', 'a') as csvFile:
            #    writer = csv.writer(csvFile)
            #    writer.writerows(self.clusters_arr)
            #csvFile.close()

            self.counter_detected_signs = 0
            #self.clusters_arr = np.empty((0,6)) # Label, X,Y,Z, counter, index
            #self.index_array = 0
            
    def publishImage(self, image, name):
        imgPub = rospy.Publisher("/"+name, Image, queue_size=2)

        imgLen = np.array(image).shape
        try:
            if len(imgLen) == 2:
                imgPub.publish(self.bridge.cv2_to_imgmsg(image, "8UC1"))
            else:
                imgPub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def poseCallback(self, data):
        poseOfDrone = data
        broadcaster = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.frame_id = 'cf1/odom'
        t.child_frame_id = 'cf1/base_link'
        t.header.stamp = rospy.Time.now()
        t.transform.translation = poseOfDrone.pose.position
        t.transform.rotation = poseOfDrone.pose.orientation
        broadcaster.sendTransform(t)

        # Camera link to Base link
        tf_c2b = TransformStamped()
        tf_c2b.header.frame_id = 'cf1/base_link'
        tf_c2b.header.stamp = rospy.Time.now()
        tf_c2b.child_frame_id = 'cf1/camera'
        tf_c2b.transform.translation.x = 0.01
        tf_c2b.transform.translation.y = 0
        tf_c2b.transform.translation.z = 0.02
        (tf_c2b.transform.rotation.x,
        tf_c2b.transform.rotation.y,
        tf_c2b.transform.rotation.z,
        tf_c2b.transform.rotation.w) = quaternion_from_euler(90*deg2rad, 180*deg2rad, 90*deg2rad) #roll, pitch, yaw
        broadcaster.sendTransform(tf_c2b)

    def getClust():
        return self.clusters_arr


rospy.init_node('perception2', anonymous=True)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
def main(args):
    # TODO:
    # Add code so that we only look at every 4th image

    #img_sub = rospy.Subscriber("/cf1/camera/image_raw", Image,imageCallback)
    #img_sub = rospy.Subscriber("/cf1/camera/image_raw/decompressed", Image,imageCallback)
    #pose_sub = rospy.Subscriber("/cf1/pose", PoseStamped, poseCallback)

    # publish the masked image with first contour extraction
    #contour_image_pub = rospy.Publisher("/contourimage", Image, queue_size=2)
    bridge = CvBridge()
    sd = signDetector(bridge)

    print("running...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    
    cluster = sd.getClust()
    print(cluster)
    """
    with open('sign_data.csv', 'w') as csvFile:
        csvWriter = csv.writer(csvFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    """

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)