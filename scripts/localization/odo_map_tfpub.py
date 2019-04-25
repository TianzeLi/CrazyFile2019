#!/usr/bin/env python
# Update the transform from map to cf1/odom 
# Using the aruco marker pose detection and compare with the pose in json file 

# Virtual Bash:
# roslaunch pras_project test2.launch
# cd dd2419_ws/src/pras_project/scripts/localization/
# python simple_localization.py 

# Current problem:

import math
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import *
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
# from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
import tf
from arucos import arucos, Rlist


x_current = 0
y_current = 0
yaw_current = 0

x_estimated = 0
y_estimated = 0 
yaw_estimated = 0


def current_pose_callback(msg):
    global x_current
    global y_current
    global yaw_current

    i = msg.pose.orientation.x
    j = msg.pose.orientation.y
    k = msg.pose.orientation.z
    r = msg.pose.orientation.w

    x_current = msg.pose.position.x
    y_current = msg.pose.position.y
    yaw_current = euler_from_quaternion((i, j, k, r))[2]
    # print(x_current)
    # print(y_current)
    # print(yaw_current)


def measurement_callback(msg):
    global x_estimated
    global y_estimated
    global yaw_estimated

    global x_current
    global y_current
    global yaw_current

    ###########################################################################
    # drone orientation estimated by aruco markers 
    aruco_relative_pose = msg.markers[0].pose.pose
    aruco_id = msg.markers[0].id
    aruco_real_pose = arucos[aruco_id-1]

    gap = [[-1.0,0.0,0.0],[0.0,0.0,1.0],[0.0,1.0,0.0]]
    R_c2d = [[0.0, 0.0, 1.0],[-1.0, 0.0, 0.0],[0.0, -1.0, 0.0]]

    i = aruco_relative_pose.orientation.x
    j = aruco_relative_pose.orientation.y
    k = aruco_relative_pose.orientation.z
    r = aruco_relative_pose.orientation.w
    R_a2c = [[1-2*(j*j + k*k), 2*(i*j - k*r), 2*(i*k + j*r)],
           [2*(i*j + k*r), 1-2*(i*i + k*k), 2*(j*k - i*r)],
           [2*(i*k - j*r), 2*(j*k + i*r), 1-2*(i*i + j*j)]]
    R_a2c = np.dot(R_a2c,gap)
    R_c2a = np.linalg.inv(R_a2c)

    R_a2m = Rlist[aruco_id-1]

    # Estimated orientation
    R_d2m_e = np.dot(R_a2m,np.dot(R_c2a, np.linalg.inv(R_c2d)))

    ###########################################################################
    # drone position estimated by aruco markers  
    t_c2d_x = 0.01
    t_c2d_y = 0.0
    t_c2d_z = 0.02

    tmp_t2 = [[t_c2d_x],[t_c2d_y],[t_c2d_z]]

    t_a2c = [[aruco_relative_pose.position.x],
             [aruco_relative_pose.position.y], 
             [aruco_relative_pose.position.z]]

    t_d2c_m = np.dot(R_d2m_e, tmp_t2)
    
    R_c2m = np.dot(R_a2m, R_c2a)
    t_c2a_m = np.dot(R_c2m, t_a2c)

    #########################################
    # Should be of the type: PoseStamped
    qw= math.sqrt(1 + R_d2m_e[0][0] + R_d2m_e[1][1] + R_d2m_e[2][2]) /2
    qx = (R_d2m_e[2][1] - R_d2m_e[1][2])/( 4 *qw)
    qy = (R_d2m_e[0][2] - R_d2m_e[2][0])/( 4 *qw)
    qz = (R_d2m_e[1][0] - R_d2m_e[0][1])/( 4 *qw)

    tmp_sum = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    qw = qw/tmp_sum
    qx = qx/tmp_sum
    qy = qy/tmp_sum
    qz = qz/tmp_sum

    x_estimated = aruco_real_pose.pose.position.x - 0.95*t_c2a_m[0] - t_d2c_m[0]
    y_estimated = aruco_real_pose.pose.position.y - 0.95*t_c2a_m[1] - t_d2c_m[1]
    yaw_estimated = euler_from_quaternion((qx, qy, qz, qw))[2]

    # print(x_estimated)
    # print(y_estimated)
    # print(yaw_estimated)
    # print("\n\n")

    tfpb = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    # t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'
    t.child_frame_id = 'cf1/odom'
    t.transform.translation.x = x_estimated - x_current
    t.transform.translation.y = y_estimated - y_current
    t.transform.translation.z = 0.0
    # -> 0;90;-90
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(0),
                                                     math.radians(0),
                                                     math.radians(yaw_estimated - yaw_current))
    # print("I am here")
    tfpb.sendTransform(t)




rospy.init_node('odo_map_tfpb')
rate = rospy.Rate(10)  # Hz

def main():

    drone_pose_origin = rospy.Subscriber('/cf1/pose', PoseStamped, current_pose_callback)
    marker_detection = rospy.Subscriber('/aruco/markers', MarkerArray, measurement_callback) 

    rospy.spin()

if __name__ == '__main__':
    main()