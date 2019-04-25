#!/usr/bin/env python
from __future__ import print_function

import roslib, rospy, tf2_ros
import sys, os, copy, math
import cv2, imutils
import numpy as np
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D, TransformStamped, PoseStamped, Vector3, Quaternion
from collections import defaultdict
from tf2_ros import StaticTransformBroadcaster
from transform import four_point_transform
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs, tf_conversions

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from keras.models import load_model, Sequential
version = '3'
#modelFolder = '/home/s/i/simoneds/dd2419_ws/src/perc_ch1/scripts/models/'
modelFolder = '/home/robot/dd2419_ws/src/PRAS_Gr6/perc_ch1/scripts/models/'

modelRect = load_model(modelFolder + 'Rectangles' + '3' + '.h5')
modelRect._make_predict_function()
modelTri = load_model(modelFolder + 'Triangles' + '3' + '.h5')
modelTri._make_predict_function()
modelCirc = load_model(modelFolder + 'Circles' + '3' + '.h5')
modelCirc._make_predict_function()

myint0 = 0
myint1 = 0
mystr = ''
comp_count = 0
bridge = CvBridge()

dataFilePath = os.getcwd() + '/src/PRAS_Gr6/perc_ch1/data/'


deg2rad = math.pi/180

class DetectorFull:
    def __init__(self, img):
        self.img = img
        self.kerasDim = (64,64)
        self.num0 = int(myint0)
        self.num1 = int(myint1)

        self.res = self.DFmain(img)

    def DFmain(self, img):
        image = img.copy()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #Grayscale image
        #Normalise image:
        normalised = gray.copy()
        norm_lower = 50
        norm_upper = 230
        cv2.normalize(gray, normalised, norm_lower, norm_upper, cv2.NORM_MINMAX)


        blurred = cv2.GaussianBlur(normalised, (5, 5), 0) #blur
        #Canny edge detection:
        canny_lowthresh = 10
        canny_ratio = 3
        canny_kernelSize = 3
        edges = cv2.Canny(blurred, canny_lowthresh, canny_lowthresh*canny_ratio, canny_kernelSize)
        imgPublish(edges,'canny')
        cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)#contours


        imgWithCnts = img.copy()
        for c in cnts:
            #Convex contour stopper:
            peri = cv2.arcLength(c, True)
            area = cv2.contourArea(c)
            approx = cv2.approxPolyDP(c, 0.1 * peri, True)
            if cv2.isContourConvex(approx) == False or area <= 65:
                continue

            #Bounding rectangle and sub-image extractor:
            minRect = cv2.minAreaRect(c)
            box = np.int0(cv2.boxPoints(minRect))
            warpedImgSection = self.boxImgTrans(img, box)

            #Color stopper for all shapes:
            maxColor = self.colorContent(warpedImgSection, outputType = 2)
            if maxColor < 0.05:
                continue

            #Shape specific stoppers:
            shape = self.classifyContourShape(c)
            if shape == 'none':
                continue
            elif shape == 'triangle':
                pass
            elif shape == 'rectangle':
                blueCont = self.colorContent(warpedImgSection, outputType = 0)[0]
                if blueCont < 0.05:
                    continue
            elif shape == 'circle':
                pass

            #Position of contour:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int((M["m10"] / M["m00"]) * 1)
                cY = int((M["m01"] / M["m00"]) * 1)
            else:
                cX = 10
                cY = 10

            #Draw contour on image and classify using NN:
            imgPublish(warpedImgSection, 'warp')
            #cv2.drawContours(image, [c], -1, (0, 255, 0), 2) #draw full contour

            signClass = self.classifySignNN(warpedImgSection, shape)
            #if signClass not in 'z_crap':
            cv2.drawContours(imgWithCnts, [box], -1, (0, 255, 0), 2) #draw box
            cv2.putText(imgWithCnts, signClass, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            #self.transformAndPublishSignTF(shape, box, signClass)


        imgPublish(imgWithCnts, 'rosbag_with_boxes')

    def classifyContourShape(self, c):
        peri = cv2.arcLength(c, True)
        area = cv2.contourArea(c)
        retval, triangle = cv2.minEnclosingTriangle(c)
        rectangle = np.int0(cv2.boxPoints(cv2.minAreaRect(c)))
        #circle_c, circle_r = cv2.minEnclosingCircle(c)

        tri_area = cv2.contourArea(triangle)
        rect_area = cv2.contourArea(rectangle)
        if len(c) >= 5:
             (ex,ey), (MA, ma), eangle = cv2.fitEllipse(c)
             ellipse_area = np.pi*MA*ma/4
        else:
            ellipse_area = 0

        areas = [tri_area, rect_area, ellipse_area]
        diff = [0,0,0]
        for i in range(0,3):
            d = area/areas[i] #ratio between areas
            diff[i] = np.sqrt(np.power(d-1,2)) #distance to ratio of 1
        mi = np.where(diff == np.amin(diff))[0][0] #find the shape with smallest distance to perfect area ratio

        shapeList = ['triangle','rectangle','circle']
        if np.amin(diff) > 0.2: #if all shapes are too different
            shape='none'
        else:
            shape = shapeList[mi]

        return shape

    def boxImgTrans(self, img, rect):
            warped = four_point_transform(img, [rect][0])
            # height, width = warped.shape[:2]
            # warped = cv2.resize(warped,(width, width), interpolation = cv2.INTER_CUBIC)
            #
            # rows,cols = warped.shape[:2]
            # rot = 0
            # M = cv2.getRotationMatrix2D(((cols-1)/2.0,(rows-1)/2.0),rot,1)
            # warped = cv2.warpAffine(warped,M,(cols,rows))
            interpolations = [cv2.INTER_NEAREST,cv2.INTER_LINEAR,cv2.INTER_AREA,cv2.INTER_CUBIC,cv2.INTER_LANCZOS4]
            i = 4
            res = cv2.resize(warped, self.kerasDim, interpolation = interpolations[i])#cv2.INTER_AREA)
            return res

    def colorContent(self, img, outputType = 0):
        #get color content in img in blue, red and yellow
        width, height = img.shape[:2]
        imgArea = width*height
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #Old colorseg values saved just in case:
        # if color == 'blue':
        #     mid = 108
        #     diff = 12
        #     lower = np.array([mid-diff,70,70])
        #     upper = np.array([mid+diff,255,255])
        # elif color == 'redOLD':
        #     mid = 6#myint0
        #     diff = 9#myint1
        #     if mid - diff < 0:
        #         lower1 = np.array([0,70,70])
        #         upper1 = np.array([mid+diff,255,255])
        #         lower2 = np.array([360+(mid-diff),70,70])
        #         upper2 = np.array([360,255,255])
        #     elif mid + diff > 360:
        #         lower1 = np.array([0,70,70])
        #         upper1 = np.array([mid-360+diff,255,255])
        #         lower2 = np.array([mid-diff,70,70])
        #         upper2 = np.array([360,255,255])
        #     else:
        #         lower1 = np.array([mid-diff,70,70])
        #         upper1 = np.array([mid+diff,255,255])
        #         lower2 = np.array([mid-diff,70,70])
        #         upper2 = np.array([mid+diff,255,255])
        # elif color == 'red':
        #     low = 16
        #     high = 330
        #     lower1 = np.array([0,70,70])
        #     upper1 = np.array([low,255,255])
        #     lower2 = np.array([high,70,70])
        #     upper2 = np.array([360,255,255])
        # elif color == 'yellow':
        #     mid = 23
        #     diff = 7
        #     lower = np.array([mid-diff,70,70])
        #     upper = np.array([mid+diff,255,255])
        # elif color == 'white':
        #     sat = 0
        #     val = 219
        #     lower = np.array([0,sat,val])
        #     upper = np.array([255,255,255])
        # elif color == 'black':
        #     sat = 255
        #     val = 98
        #     lower = np.array([0,0,0])
        #     upper = np.array([255,sat,val])
        # elif color == 'green':
        #     print('ERROR: please enter a creative color')
        # else: #if something else, dont segment
        #     print('ERRROR: color not found')

        # define range of the color we look for in the HSV space
        #colorizer.org
        cols = ['blue', 'red', 'yellow']
        res = [0,0,0]
        for j in range(0,3):
            color = cols[j]
            if color == 'blue':
                mid = 108
                diff = 12
                lower = np.array([mid-diff,70,70])
                upper = np.array([mid+diff,255,255])
            elif color == 'red':
                low = 16
                high = 330
                lower1 = np.array([0,70,70])
                upper1 = np.array([low,255,255])
                lower2 = np.array([high,70,70])
                upper2 = np.array([360,255,255])
            elif color == 'yellow':
                mid = 23
                diff = 7
                lower = np.array([mid-diff,70,70])
                upper = np.array([mid+diff,255,255])
            # Apply mask:
            if color == 'red':
                mask1 = cv2.inRange(hsv, lower1, upper1)
                mask2 = cv2.inRange(hsv, lower2, upper2)
                masked1 = cv2.bitwise_and(img, img, mask= mask1)
                imgPublish(masked1,'masked1')
                masked2 = cv2.bitwise_and(img, img, mask= mask2)
                imgPublish(masked1,'masked2')
                masked = cv2.bitwise_or(masked1, masked2)
                imgPublish(masked1,'masked')
            else:
                # Threshold the HSV image to get only the pixels in range
                mask = cv2.inRange(hsv, lower, upper)
                masked = cv2.bitwise_and(img, img, mask= mask)

            # convert the resized image to grayscale and threshold it
            gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
            thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]


            #res[j] = float(np.sum(thresh)/imgArea
            res[j] = float(cv2.countNonZero(thresh)) / imgArea
            #print(str(np.sum(thresh)))
        if outputType == 0:
            return res
        elif outputType == 1:
            ci = np.where(res == np.amax(res))[0][0]
            return cols[ci]
        elif outputType == 2:
            max = np.amax(res)
            return max

    def classifySignNN(self, img, shape):
        #input: 64x64 image
        #output: classification string

        if shape == 'rectangle':
            classNames = ['airport', 'residential', 'z_crap']
            model = modelRect
        elif shape == 'circle':
            classNames = ['follow_left','follow_right','no_bicycle','no_heavy_truck','no_parking','no_stopping_and_parking','stop','z_crap']
            model = modelCirc
        elif shape == 'triangle':
            classNames = ['dangerous_curve_left','dangerous_curve_right','junction','road_narrows_from_left','road_narrows_from_right','roundabout_warning','z_crap']
            model = modelTri

        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img2 = np.expand_dims(np.array(imgRGB),0)
        pred = model.predict(img2) #prediction between classes
        p = np.where(pred == np.amax(pred))[1][0] #index of mose likely class
        out = classNames[p]

        #imgplot = plt.imshow(img)
        #plt.title(out)
        #plt.show()
        return out

    def transformAndPublishSignTF(self, shape, box, signClass):
        #Input: sign and shape
        #Output: Publishes a transfer-frame of the sign
        #Camera parameters
        cameraMatrix = np.array([231.250001, 0.000000, 320.519378, 0.000000, 231.065552, 240.631482, 0.000000, 0.000000, 1.000000])
        cameraMatrix = cameraMatrix.reshape(3,3)
        distCoeffs = np.array([0.061687, -0.049761, -0.008166, 0.004284, 0.000000])

        if shape == 'triangle':
            width_sign = 0.18
            height_sign = 0.16
        elif shape == 'rectangle':
            width_sign = 0.18
            height_sign = 0.115
        elif shape == 'circle':
            width_sign = 0.18
            height_sign = 0.18

        height = height_sign
        width = width_sign

        left_top_sign = [[0], [-width/2], [height/2]]
        right_top_sign = [[0], [width/2], [height/2]]
        left_bottom_sign = [[0], [-width/2], [-height/2]]
        right_bottom_sign = [[0], [width/2], [-height/2]]

        objectPoints = np.array([left_top_sign, right_top_sign, left_bottom_sign, right_bottom_sign])

        # top_left = [[top_left.x], [top_left.y]]
        # top_right = [[top_right.x], [top_right.y]]
        # bottom_left = [[bottom_left.x], [bottom_left.y]]
        # bottom_right = [[bottom_right.x], [bottom_right.y]]
        # imagePoints = np.array([top_left, top_right, bottom_left, bottom_right])
        imagePoints = np.float32(box)

        (success, rotation_vector, translation_vector) = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)

        if success:
            (x, y, z, w) = quaternion_from_euler(rotation_vector[0], rotation_vector[1], rotation_vector[2])
            translation = Vector3(translation_vector[0], translation_vector[1], translation_vector[2])
            rotation = Quaternion(x, y, z, w)

            br = tf2_ros.StaticTransformBroadcaster()
            t = PoseStamped()
            t.header.frame_id = 'cf1/camera'
            #t.child_frame_id = 'cf1/' + signClass
            t.header.stamp = rospy.Time()

            t.pose.position.x = translation_vector[0]
            t.pose.position.y = translation_vector[1]
            t.pose.position.z = translation_vector[2]

            t.pose.orientation.x = rotation.x
            t.pose.orientation.y = rotation.y
            t.pose.orientation.z = rotation.z
            t.pose.orientation.w = rotation.w

            trans = tf_buf.transform(t,'cf1/odom') #transform marker pose from camera to odom frame

            tOut = TransformStamped()
            tOut.header.frame_id = 'map'
            tOut.child_frame_id = 'cf1/' + signClass
            tOut.transform.translation = trans.pose.position
            tOut.transform.rotation = trans.pose.orientation

            br.sendTransform(tOut)
        else:
            print('Localisation is not possible')

def poseCallback(data):
    dronePose = data
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.frame_id = 'cf1/odom'
    t.child_frame_id = 'cf1/base_link'
    t.header.stamp = rospy.Time.now()
    t.transform.translation = dronePose.pose.position
    t.transform.rotation = dronePose.pose.orientation
    br.sendTransform(t)

    tf_c2b = TransformStamped() #camera to baselink
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
    br.sendTransform(tf_c2b)


def imgCallback2(data):
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    DF = DetectorFull(img)

def imgPublish(img, pubname):
    image_pub = rospy.Publisher("/"+pubname, Image, queue_size=2)


    imga = np.array(img).shape
    try:
        if len(imga) == 2:
            image_pub.publish(bridge.cv2_to_imgmsg(img, "8UC1"))
        else:
            image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
        print(e)

def imgCallback(data):
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    COL = ['blue']
    NBLUR = [1, 1]
    harshness = 0.041
    invert = 0

    #List shapens in image from img and color
    global comp_count
    comp_count += 1
    hz = 1

    if np.mod(comp_count, hz) == 0:
        for i in range(0,len(COL)):
            col = COL[i]
            nblur = NBLUR[i]
            SD = ShapeDetector(col, img, harshness, invert, nblur)
            res = SD.res
            bw = SD.bw
            #imgPublish(res, '_shapes')
            #imgPublish(bw, '_seg')
            imgPublish(res, col+'_shapes')
            imgPublish(bw, col+'_seg')

class myCallbacker:
    def myintcb0(self,data):
        global myint0
        myint0 = data.data
    def myintcb1(self,data):
        global myint1
        myint1 = data.data
    def mystrcb(self,data):
        global mystr
        mystr = data.data

#Main:
rospy.init_node('perc_ch_full', anonymous=True)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
def main(args):

    img_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, imgCallback2)
    #img_sub = rospy.Subscriber("/cf1/camera/image_raw/decompressed", Image, imgCallback2)
    pose_sub = rospy.Subscriber("/cf1/pose", PoseStamped, poseCallback)

    Cb = myCallbacker()
    myint_sub0 = rospy.Subscriber("/myint0", Int32, Cb.myintcb0)
    myint_sub1 = rospy.Subscriber("/myint1", Int32, Cb.myintcb1)
    mystr_sub = rospy.Subscriber("/mystr", String, Cb.mystrcb)

    print("running...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
