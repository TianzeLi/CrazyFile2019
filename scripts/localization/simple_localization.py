#!/usr/bin/env python
# Update the transform from cf1/odom to cf1/base_link 
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
from arucosII import arucos, Rlist


#########################################
# The estimated pose in in this variable!
drone_pose = None
#########################################
tf_buf = None
R_d2m = np.eye(3)
drone_orientation = None
pose_buff = []
x_lastest = None
y_lastest = None
drone_z = None


def predict_callback(msg):
    global R_d2m
    global drone_orientation
    global pose_buff
    global drone_pose
    global x_lastest
    global y_lastest
    global drone_z

    drone_z = msg.pose.position.z

    i = msg.pose.orientation.x
    j = msg.pose.orientation.y
    k = msg.pose.orientation.z
    r = msg.pose.orientation.w
    drone_orientation =[i,j,k,r]

    # Might be wrong here
    R_d2m = [[1-2*(j*j + k*k), 2*(i*j - k*r), 2*(i*k + j*r)],
           [2*(i*j + k*r), 1-2*(i*i + k*k), 2*(j*k - i*r)],
           [2*(i*k - j*r), 2*(j*k + i*r), 1-2*(i*i + j*j)]]
    # print(R_m2d)
    # R_d2m = np.linalg.inv(R_m2d)
    # print('Modified by perdict')
    # print(R_d2m)
    # print('\n\n')

    if x_lastest == None:
        x_lastest = msg.pose.position.x
        y_lastest = msg.pose.position.y
    if drone_pose == None:
        drone_pose = msg

    delta_x = msg.pose.position.x - x_lastest
    delta_y = msg.pose.position.y - y_lastest
    # drone_pose.header.stamp = rospy.Time.now()
    drone_pose.header.stamp = msg.header.stamp
    drone_pose.header.frame_id = "map"
    drone_pose.pose.position.x =  drone_pose.pose.position.x + delta_x
    drone_pose.pose.position.y = drone_pose.pose.position.y + delta_y
    drone_pose.pose.position.z = msg.pose.position.z
    drone_pose.pose.orientation.w = drone_orientation[3]
    drone_pose.pose.orientation.x = drone_orientation[0]
    drone_pose.pose.orientation.y = drone_orientation[1]
    drone_pose.pose.orientation.z = drone_orientation[2]

    x_lastest = msg.pose.position.x
    y_lastest = msg.pose.position.y

    position_estimated.publish(drone_pose)


def measurement_callback(msg):
    global tf_buf
    global R_d2m
    global drone_orientation
    global arucos
    global drone_pose
    global drone_z

    gap = [[-1.0,0.0,0.0],[0.0,0.0,1.0],[0.0,1.0,0.0]]
    R_c2d = [[0.0, 0.0, 1.0],[-1.0, 0.0, 0.0],[0.0, -1.0, 0.0]]

    ###########################################################################
    # transform from camera_link to base_link, working fine!
    t_c2d_x = 0.01
    t_c2d_y = 0.0
    t_c2d_z = 0.02

    tmp_t2 = [[t_c2d_x],[t_c2d_y],[t_c2d_z]]
    t_d2c_m = np.dot(R_d2m, tmp_t2)
    ###########################################################################
    # aruco pose respective to carema_link 
    aruco_relative_pose = msg.markers[0].pose.pose
    aruco_id = msg.markers[0].id
    aruco_real_pose = arucos[aruco_id-1]

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
    # EXAMPLE
    R_a2m_e = np.dot(R_d2m, np.dot(R_c2d,R_a2c))
    # print(R_a2m_e)
    # print('\n\n')
    

    # R_c2a = np.dot(np.dot(Rz_c2d, Ry_c2d),np.dot(R_d2m,np.linalg.inv(R_a2m)))
    t_a2c = [[aruco_relative_pose.position.x],
             [aruco_relative_pose.position.y], 
             [aruco_relative_pose.position.z]]
    # Estimated orientation
    # print()
    R_d2m_e = np.dot(R_a2m,np.dot(R_c2a, np.linalg.inv(R_c2d)))
    # So far correct
    ############################################################################

    # Using R_d2m from estimation     
    # R_c2m = np.dot(np.dot(Rz_c2d, Ry_c2d), R_d2m_e)
    # R_c2m = np.dot(np.linalg.inv(R_c2a), R_a2m)
    R_c2m = np.dot(R_a2m, R_c2a)

    t_c2a_m = np.dot(R_c2m, t_a2c)

    print(t_c2a_m)
    print('\n\n')

    #########################################
    # Should be of the type: PoseStamped
    drone_pose = PoseStamped()
    # drone_pose.header.stamp = rospy.Time.now()
    drone_pose.header.stamp = msg.header.stamp
    drone_pose.header.frame_id = "map"

    qw= math.sqrt(1 + R_d2m_e[0][0] + R_d2m_e[1][1] + R_d2m_e[2][2]) /2
    qx = (R_d2m_e[2][1] - R_d2m_e[1][2])/( 4 *qw)
    qy = (R_d2m_e[0][2] - R_d2m_e[2][0])/( 4 *qw)
    qz = (R_d2m_e[1][0] - R_d2m_e[0][1])/( 4 *qw)

    # drone_pose.pose.orientation.w = drone_orientation[3]
    # drone_pose.pose.orientation.x = drone_orientation[0]
    # drone_pose.pose.orientation.y = drone_orientation[1]
    # drone_pose.pose.orientation.z = drone_orientation[2]

    drone_pose.pose.orientation.w = qw
    drone_pose.pose.orientation.x = qx
    drone_pose.pose.orientation.y = qy
    drone_pose.pose.orientation.z = qz
    position_estimated.publish(drone_pose)

    # # better confirm the detected orientation of aruco markers first!
    # R_d2m = np.dot(R_d2c,np.dot(R_c2a,R_a2m))
    # R_m2d = np.linalg.inv(R_d2m)
    # print('LATER R_d2m')
    # print(R_d2m)
    # print('\n\n')
    # # print(R_m2d)

    drone_pose.pose.position.x = aruco_real_pose.pose.position.x - t_c2a_m[0] - t_d2c_m[0]
    drone_pose.pose.position.y = aruco_real_pose.pose.position.y - t_c2a_m[1] - t_d2c_m[1]
    drone_pose.pose.position.z = drone_z

    print(drone_pose)


















rospy.init_node('measurement_from_aruco')
rate = rospy.Rate(10)  # Hz
position_estimated  = rospy.Publisher('/localization', PoseStamped, queue_size = 3)

def main():
    global tf_buf 

    # tf from base_link to camera_link only valid for simulation! 
    tfpb = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    # t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'cf1/base_link'
    t.child_frame_id = 'cf1/camera_link'
    t.transform.translation.x = 0
    t.transform.translation.y = 0.01
    t.transform.translation.z = 0.02
    # -> 0;90;-90
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(0),
                                                     math.radians(90),
                                                     math.radians(-90),'rzyz')
    tfpb.sendTransform(t)
    ############################################
    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)
    ############################################
    drone_pose_origin = rospy.Subscriber('/cf1/pose', PoseStamped, predict_callback)
    marker_detection = rospy.Subscriber('/aruco/markers', MarkerArray, measurement_callback)

    rospy.spin()

if __name__ == '__main__':
    main()



# # Marker message type: 

# header: 
#   seq: 585
#   stamp: 
#     secs: 636
#     nsecs: 160000000
#   frame_id: "camera_link"
# markers: 
#   - 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 636
#         nsecs: 160000000
#       frame_id: "camera_link"
#     id: 7
#     pose: 
#       pose: 
#         position: 
#           x: -0.153540790081
#           y: 0.418781787157
#           z: 0.704935073853
#         orientation: 
#           x: -0.0947737273034
#           y: 0.000769182143555
#           z: 0.995450843381
#           w: 0.00974512100507
#       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     confidence: 1.0
# ---


# #########################################
# # RPY to convert: 90deg, 0, -90deg
# q = quaternion_from_euler(1.5707, 0, -1.5707)
# #########################################
# # To apply the rotation of one quaternion to a pose, simply multiply the previous quaternion of the pose 
# # by the quaternion representing the desired rotation. The order of this multiplication matters. 
# q_orig = quaternion_from_euler(0, 0, 0)
# q_rot = quaternion_from_euler(pi, 0, 0)
# q_new = quaternion_multiply(q_rot, q_orig)
# print q_new
# #########################################
# # An example to get the relative rotation from the previous robot pose to the current robot pose: 
# q1_inv[0] = prev_pose.pose.orientation.x
# q1_inv[1] = prev_pose.pose.orientation.y
# q1_inv[2] = prev_pose.pose.orientation.z
# q1_inv[3] = -prev_pose.pose.orientation.w # Negate for inverse

# q2[0] = current_pose.pose.orientation.x
# q2[1] = current_pose.pose.orientation.y
# q2[2] = current_pose.pose.orientation.z
# q2[3] = current_pose.pose.orientation.w
 
# qr = tf.transformations.quaternion_multiply(q2, q1_inv)



# Codes for reference
    # r = math.sqrt(1.05 + R_a2m_e[0][0] + R_a2m_e[1][1] + R_a2m_e[2][2]) /2
    # i = (R_a2m_e[2][1] - R_a2m_e[1][2])/( 4 *r)
    # j = (R_a2m_e[0][2] - R_a2m_e[2][0])/( 4 *r)
    # k = (R_a2m_e[1][0] - R_a2m_e[0][1])/( 4 *r)
    # print(euler_from_quaternion([i,j,k,r],'sxyz'))