#!/usr/bin/env python

# ArUco Subscriber
# This code detects ArUco markers and transform them to base_link.
# To be used to know the relative pose between the drone and the ArUco marker
# to determine an estimated position of the drone.

import sys
import math
import json

import rospy
import tf2_ros
import tf_conversions
import tf2_geometry_msgs
import aruco_msgs
import tf2_msgs
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped, Quaternion
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo


def markerDetectedCallback(msg):
    global marker, tf_buf
    n = len(msg.markers)
    broadcaster = tf2_ros.TransformBroadcaster()

    for i in range(0, n):
        marker = msg.markers[i]
        markerId = marker.id
        if markerId != 0:
            transform_from_marker(marker, tf_buf, broadcaster)
            #broadcaster.sendTransform(tfFinish)
    

def transform_from_marker():
    # Marker pose in camera_link
    markerPosition = m.pose.pose.position
    markerOrientation = m.pose.pose.orientation

    markerOut = PoseStamped()
    markerOut.pose = m.pose.pose
    markerOut.header.frame_id = 'cf1/camera'
    markerOut.header.stamp = rospy.Time()

    tfC2B = TransformStamped()
    tfC2B.header.frame_id = 'cf1/base_link'
    tfC2B.header.stamp = rospy.Time.now()

    tfC2B.child_frame_id = 'cf1/camera'
    tfC2B.transform.translation.x = 0.01
    tfC2B.transform.translation.y = 0
    tfC2B.transform.translation.z = 0.02
    (tfC2B.transform.rotation.x,
     tfC2B.transform.rotation.y,
     tfC2B.transform.rotation.z,
     tfC2B.transform.rotation.w) = quaternion_from_euler(90*rad2deg,
                                                        180*rad2deg,
                                                        90*rad2deg)
    broadcaster.sendTransform(tfC2B)
    #rospy.sleep(0.0001)
    poseFinish = tf_buf.transform(markerOut, 'cf1/odom')

    print('pose = ', poseFinish.pose.position)
    print('orientation = ', poseFinish.pose.orientation)

def listener():
    rospy.Subscriber('/aruco/markers', MarkerArray, markerDetectedCallback)

rospy.init_node('detectmarker', anonymous=True)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main():
    listener()
    rospy.spin()

if __name__ == '__main__':
    main()