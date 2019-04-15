#!/usr/bin/env python

# Take the ArUco marker detections, transform them into the map frame in a node 
# and publish a tf from map to /aruco/detectedX where X corresponds to the id of the marker. 

# Virtual Bash:
# roslaunch dd2419_simulation simulation.launch
# roslaunch dd2419_simulation aruco.launch gui:=false
# rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map cf1/odom
# roslaunch dd2419_launch base.launch ch:=92



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

tf_buf = None

def pose_callback(msg):
    global tf_buf

    # rospy.loginfo('New pose set:\n%s', msg)

    # Should be of the type: PoseStamped
    base_pose = PoseStamped()
    base_pose.header.stamp = rospy.Time.now()
    base_pose.header.frame_id = "cf1/camera_link"
    base_pose.pose = msg.markers[0].pose.pose

    marker_tunnel = '/aruco/detected'
    marker_tunnel = marker_tunnel + str(msg.markers[0].id)
    # print(marker_tunnel)


    # tf_buf.can_transform
    if not tf_buf.can_transform(base_pose.header.frame_id, 'map', base_pose.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % base_pose.header.frame_id)
        return

    goal_pose = tf_buf.transform(base_pose, 'map')
    marker_tfpb = tf2_ros.StaticTransformBroadcaster()
    mt = TransformStamped()
    # mt.header.stamp = rospy.Time.now()
    mt.header.frame_id = "map"
    mt.child_frame_id = marker_tunnel
    mt.transform.translation.x = goal_pose.pose.position.x
    mt.transform.translation.y = goal_pose.pose.position.y
    mt.transform.translation.z = goal_pose.pose.position.z
    mt.transform.rotation.x = goal_pose.pose.orientation.x
    mt.transform.rotation.y = goal_pose.pose.orientation.y
    mt.transform.rotation.z = goal_pose.pose.orientation.z
    mt.transform.rotation.w = goal_pose.pose.orientation.w

    marker_tfpb.sendTransform(mt)



def main():
    global tf_buf 

    rospy.init_node('aruco_tf')
    rate = rospy.Rate(10000)  # Hz
    
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

    marker_detection = rospy.Subscriber('/aruco/markers', MarkerArray, pose_callback)


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
