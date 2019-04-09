#!/usr/bin/env python

import numpy as np
import math
import rospy
import json
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
import tf


# Better write into classes, then first generate 'len(signs)' instances
# Potiential mistake: variables not arrays
# The current version can only get the pose of one sign 



d1 = []
d2 = []
d3 = []
value = []



def pose_tf(pose, u, v, sign_no):

    global d1
    global d2
    global value

    t_c2d_x = 0
    t_c2d_y = 0.01
    t_c2d_z = 0.02

    Ry_c2d = [[0,0,1.0],[0,1.0,0],[-1.0,0,0]]
    Rz_c2d = [[0,1.0,0],[-1.0,0,0],[0,0,1.0]]

    i = pose.pose.orientation.x
    j = pose.pose.orientation.y
    k = pose.pose.orientation.z
    r = pose.pose.orientation.w

    x_c = pose.pose.position.x + t_c2d_x
    y_c = pose.pose.position.y + t_c2d_y
    z_c = pose.pose.position.z + t_c2d_z
    C_m = [[x_c],[y_c],[z_c]]

    # Transform quaternions to rotation matrix, right equation but wrong result
    # https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    R_d = [[1-2*(j*j + k*k), 2*(i*j - k*r), 2*(i*k + j*r)],
           [2*(i*j + k*r), 1-2*(i*k + k*k), 2*(j*k - i*r)],
           [2*(i*k - j*r), 2*(j*k + i*r), 1-2*(i*i + j*j)]]

    #########################################
    # Extrinsic parameters
    R = np.dot(np.dot(Rz_c2d, Ry_c2d),R_d)
    t_array = np.dot(R, C_m)

    t = []
    t.append(-t_array[0][0])
    t.append(-t_array[1][0]) 
    t.append(-t_array[2][0]) 
    print R
    print C_m
    #########################################

    #########################################
    # Intrinsic parameters
    cx = 320.0
    cy = 240.0
    fx = 231.0
    fy = 231.0
    #########################################
    
    x_z = (u - cx)/fx
    y_z = (v - cy)/fy

    ########################################################################################
    # Only when the new point is not an outlier

    d1.append(x_z*R[2][0] - R[0][0])
    d2.append(x_z*R[2][1] - R[0][1])
    d3.append(x_z*R[2][2] - R[0][2])    
    value.append(-(x_z*t[2] - t[0]))

    d1.append(y_z*R[2][0] - R[1][0])
    d2.append(y_z*R[2][1] - R[1][1])
    d3.append(y_z*R[2][2] - R[1][2])    
    value.append(-(y_z*t[2] - t[1]))

    # Regression part
    # https://stackoverflow.com/questions/11479064/multiple-linear-regression-in-python
    if len(d1) > 3:
        D = np.c_[d1,d2,d3]
        # D = D.T # transpose so input vectors are along the rows

        # print D
        # print value
        result = np.linalg.lstsq(D,value)[0]
        print(result)
    ########################################################################################

    # return [X,Y,Z]


# For testing's use
def main():

    p1 = PoseStamped()
    p1.pose.position.x = 0
    p1.pose.position.y = 0
    p1.pose.position.z = 0
    p1.pose.orientation.x = 0 
    p1.pose.orientation.y = 0 
    p1.pose.orientation.z = 0 
    p1.pose.orientation.w = 1

    p2 = PoseStamped()
    p2.pose.position.x = 0
    p2.pose.position.y = 0
    p2.pose.position.z = 4
    p2.pose.orientation.x = 0 
    p2.pose.orientation.y = 0 
    p2.pose.orientation.z = 0 
    p2.pose.orientation.w = 1
    pose_tf(p1, 320, 240, 1)
    pose_tf(p2, 320, 200, 1)



if __name__ == '__main__':
    main()


# drone pose example
# header: 
#   seq: 174640
#   stamp: 
#     secs: 1551712142
#     nsecs: 258133362
#   frame_id: "map"
# pose: 
#   position: 
#     x: 0.715396344662
#     y: 0.612033188343
#     z: 0.518254101276
#   orientation: 
#     x: -0.0431273132563
#     y: 0.0256682503968
#     z: 0.635533869267
#     w: -0.770440161228
