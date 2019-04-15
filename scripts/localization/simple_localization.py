#!/usr/bin/env python

# Update the transform from cf1/odom to cf1/base_link 
# Using the aruco marker pose detection and compare with the pose in json file 

# Virtual Bash:
# roslaunch pras_project nav_challenge.launch
# cd dd2419_ws/src/pras_project/scripts/localization/
# python simple_localization.py 


# Current problem: ???


import math
import numpy as np
import rospy
import json
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
import tf
from arucos import arucos, Rlist

#########################################
# The estimated pose in in this variable!
# The difference of wrong one and "correct" one is drone_pose - wrong one,
# Just plus it to the goal you send to cmd_position. 
drone_pose = None
#########################################

tf_buf = None
R_d2m = np.eye(3)
drone_orientation = None
pose_buff = []
x_lastest = None
y_lastest = None


def d_pose_callback(msg):
    global R_d2m
    global drone_orientation
    global pose_buff
    global drone_pose
    global x_lastest
    global y_lastest 

    i = msg.pose.orientation.x
    j = msg.pose.orientation.y
    k = msg.pose.orientation.z
    r = msg.pose.orientation.w

    R_d2m = [[1-2*(j*j + k*k), 2*(i*j - k*r), 2*(i*k + j*r)],
           [2*(i*j + k*r), 1-2*(i*k + k*k), 2*(j*k - i*r)],
           [2*(i*k - j*r), 2*(j*k + i*r), 1-2*(i*i + j*j)]]

    R_d2m = np.linalg.inv(R_d2m)

    drone_orientation =[i,j,k,r]
    if x_lastest == None:
        x_lastest = msg.pose.position.x
        y_lastest = msg.pose.position.y
    if drone_pose == None:
        drone_pose = msg

    delta_x = msg.pose.position.x - x_lastest
    delta_y = msg.pose.position.y - y_lastest

    drone_pose.header.stamp = rospy.Time.now()
    drone_pose.header.frame_id = "map"
    drone_pose.pose.position.x =  drone_pose.pose.position.x + delta_x
    drone_pose.pose.position.y = drone_pose.pose.position.y + delta_y
    drone_pose.pose.position.z = msg.pose.position.x

    drone_pose.pose.orientation.w = drone_orientation[3]
    drone_pose.pose.orientation.x = drone_orientation[0]
    drone_pose.pose.orientation.y = drone_orientation[1]
    drone_pose.pose.orientation.z = drone_orientation[2]

    x_lastest = msg.pose.position.x
    y_lastest = msg.pose.position.y

    # print("POSE")
    # print(drone_pose.pose.position.x - msg.pose.position.x)
    # print(drone_pose.pose.position.y - msg.pose.position.y)
    # # print(drone_pose.pose.position.z - msg.pose.position.x)
    # print("\n")

    position_estimated.publish(drone_pose)



def pose_callback(msg):
    global tf_buf
    global R_d2m
    global drone_orientation
    global arucos
    global drone_pose

    ###########################################################################
    # transform from camera_link to base_link, working fine!
    t_c2d_x = 0.01
    t_c2d_y = 0
    t_c2d_z = 0.02

    Ry_c2d = [[0,0,1.0],[0,1.0,0],[-1.0,0,0]]
    Rz_c2d = [[0,1.0,0],[-1.0,0,0],[0,0,1.0]]

    R_d2c = np.linalg.inv(np.dot(Rz_c2d, Ry_c2d))
    tmp_t2 = [[t_c2d_x],[t_c2d_y],[t_c2d_z]]
    t_d2c_m = np.dot(np.linalg.inv(R_d2m), tmp_t2)
    # t_d2c = -tmp_t2
    # print(t_d2c_m)
    ###########################################################################
    # aruco pose respective to carema_link 
    aruco_relative_pose = msg.markers[0].pose.pose
    aruco_id = msg.markers[0].id
    aruco_real_pose = arucos[aruco_id-1]

    # i = aruco_relative_pose.orientation.x
    # j = aruco_relative_pose.orientation.y
    # k = aruco_relative_pose.orientation.z
    # r = aruco_relative_pose.orientation.w

    # tmp_sum = math.sqrt(i**2 + j**2 + k**2 + r**2)
    # i = i/tmp_sum
    # j = j/tmp_sum
    # k = k/tmp_sum
    # r = r/tmp_sum

    # R_a2m = [[1-2*(j*j + k*k), 2*(i*j - k*r), 2*(i*k + j*r)],
    #        [2*(i*j + k*r), 1-2*(i*k + k*k), 2*(j*k - i*r)],
    #        [2*(i*k - j*r), 2*(j*k + i*r), 1-2*(i*i + j*j)]]
    # #No.8
    # R_a2m = [[0, 0, -1],[0.707, -0.707, 0],[-0.707, -0.707, 0]]
    # #No.15
    # # R_a2m = [[-1, 0, 0],[0, -1, 0],[0, 0, 1]]

    R_a2m = Rlist[aruco_id-1]
    # R_a2m = np.linalg.inv(R_a2m)
    # print(R_a2m)    

    # R_c2a = np.dot(np.dot(Rz_c2d, Ry_c2d),np.dot(R_d2m,np.linalg.inv(R_a2m)))
    tmp_t = [[aruco_relative_pose.position.x],
             [aruco_relative_pose.position.y], 
             [aruco_relative_pose.position.z]]
    # cd, dm
    # R_c2m = np.dot(R_c2a, R_a2m)
    
    R_c2m = np.dot(np.dot(Rz_c2d, Ry_c2d), R_d2m)

    # t_c2a = np.dot(R_c2a, tmp_t)
    t_c2a_m = np.dot(np.linalg.inv(R_c2m), tmp_t)

    # t_c2a_m = np.dot(np.linalg.inv(R_a2m),t_c2a_m)
    # print(t_c2a_m)

    #########################################
    # rospy.loginfo('New pose set:\n%s', msg)

    # Should be of the type: PoseStamped
    drone_pose = PoseStamped()
    drone_pose.header.stamp = rospy.Time.now()
    drone_pose.header.frame_id = "map"
    drone_pose.pose.position.x = aruco_real_pose.pose.position.x + t_c2a_m[0] - t_d2c_m[0]
    drone_pose.pose.position.y = aruco_real_pose.pose.position.y + t_c2a_m[1] - t_d2c_m[1]
    drone_pose.pose.position.z = aruco_real_pose.pose.position.z - t_c2a_m[2] - t_d2c_m[2]

    drone_pose.pose.orientation.w = drone_orientation[3]
    drone_pose.pose.orientation.x = drone_orientation[0]
    drone_pose.pose.orientation.y = drone_orientation[1]
    drone_pose.pose.orientation.z = drone_orientation[2]

    position_estimated.publish(drone_pose)

    # print("POSE")
    # print(drone_pose.pose.position.x)
    # print(drone_pose.pose.position.y)
    # print(drone_pose.pose.position.z)
    # print("\n\n\n\n")




    # tmp_R = np.dot(R_b2c, R_c2a)

    # i = aruco_real_pose.pose.orientation.x
    # j = aruco_real_pose.pose.orientation.y
    # k = aruco_real_pose.pose.orientation.z
    # r = aruco_real_pose.pose.orientation.w

    # R_a = [[1-2*(j*j + k*k), 2*(i*j - k*r), 2*(i*k + j*r)],
    #        [2*(i*j + k*r), 1-2*(i*k + k*k), 2*(j*k - i*r)],
    #        [2*(i*k - j*r), 2*(j*k + i*r), 1-2*(i*i + j*j)]]
    # R_drone = np.dot(tmp_R, R_a2m)
    # drone_pose.orientation.w = sqrt((1 + R_drone[0][0] + R_drone[1][1] + R_drone[2][2]))/2.0
    # drone_pose.orientation.x = (R_drone[2][1] - R_drone[1][2])/(4*drone_pose.orientation.w)
    # drone_pose.orientation.y = (R_drone[0][2] - R_drone[2][0])/(4*drone_pose.orientation.w)
    # drone_pose.orientation.z = (R_drone[1][0] - R_drone[0][1])/(4*drone_pose.orientation.w)    



    # marker_tunnel = '/aruco/detected'
    # marker_tunnel = marker_tunnel + str(msg.markers[0].id)
    # # print(marker_tunnel)


    # # tf_buf.can_transform
    # if not tf_buf.can_transform(base_pose.header.frame_id, 'map', base_pose.header.stamp):
    #     rospy.logwarn_throttle(5.0, 'No transform from %s to map' % base_pose.header.frame_id)
    #     return

    # goal_pose = tf_buf.transform(base_pose, 'map')

    # # publish the transform based on the pose of aruco markers
    # marker_tfpb = tf2_ros.StaticTransformBroadcaster()
    # mt = TransformStamped()
    # # mt.header.stamp = rospy.Time.now()
    # mt.header.frame_id = "map"
    # mt.child_frame_id = marker_tunnel
    # mt.transform.translation.x = goal_pose.pose.position.x
    # mt.transform.translation.y = goal_pose.pose.position.y
    # mt.transform.translation.z = goal_pose.pose.position.z
    # mt.transform.rotation.x = goal_pose.pose.orientation.x
    # mt.transform.rotation.y = goal_pose.pose.orientation.y
    # mt.transform.rotation.z = goal_pose.pose.orientation.z
    # mt.transform.rotation.w = goal_pose.pose.orientation.w

    # marker_tfpb.sendTransform(mt)


rospy.init_node('measurement_from_aruco')
rate = rospy.Rate(10000)  # Hz

drone_pose_origin = rospy.Subscriber('/cf1/pose', PoseStamped, d_pose_callback)
position_estimated  = rospy.Publisher('/localization', PoseStamped, queue_size=2)

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

    drone_pose_origin = rospy.Subscriber('/cf1/pose', PoseStamped, d_pose_callback)
    marker_detection = rospy.Subscriber('/aruco/markers', MarkerArray, pose_callback)

    # position_cmd.publish(drone_pose)



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
