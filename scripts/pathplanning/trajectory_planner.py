#!/usr/bin/python2.7
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
import visualization_msgs
import copy
import planners.astar

from move import Move
from state import State
from robot import Robot
from map import Map

#Global variables
k = 1
dr_pose = None

#Define posestamped message for waypoints
posemsg = PoseStamped()
posemsg.header.frame_id = 'cf1/odom'


class TrajectoryPlanner:
    def __init__(self):
        self.map = None
        self.start = None
        self.goal = None

        self.moves = [Move(0.02, 0),  # forward
                      Move(-0.02, 0),  # back
                      Move(0, 1.5708),  # turn left 90
                      Move(0, -1.5708)] # turn right 90
        self.robot = Robot(0.101, 0.101)
        self.is_working = False # Replace with mutex after all

        self.map_subscriber = rospy.Subscriber("map", OccupancyGrid, self.new_map_callback)
        self.start_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.new_start_callback)
        self.goal_subscriber = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.new_goal_callback)
        self.local_subscriber = rospy.Subscriber("/localization", PoseStamped, self.local_callback)

        self.path_publisher = rospy.Publisher("trajectory", MarkerArray, queue_size=3)
        self.pose_publisher = rospy.Publisher("debug_pose", PoseStamped, queue_size=3)

        # what will be there. A module goes into variable. Isn't it too much memory consumption. Maybe I should assign function replan() to this variable?
        self.planner = planners.astar.replan

    def local_callback(self, d_pose):
    	global dr_pose
    	dr_pose = d_pose


    def ready_to_plan(self):
        return self.map is not None and self.start is not None and self.goal is not None

    def new_goal_callback(self, goal_pose):
    	posemsg.header.stamp = goal_pose.header.stamp
        if not self.is_working:
            self.is_working = True
            new_goal = State.from_pose(goal_pose.pose)
            if self.map is not None and self.map.is_allowed(new_goal, self.robot):
                self.goal = new_goal
                rospy.loginfo("New goal was set")
                if self.ready_to_plan():
                    self.replan()
            else:
                rospy.logwarn("New goal is bad or no map available")

            self.is_working = False

    def new_start_callback(self, start_pose):
        if not self.is_working:
            self.is_working = True
            new_start = State.from_pose(start_pose.pose.pose)
            if self.map is not None and self.map.is_allowed(new_start, self.robot):
                self.start = new_start
                rospy.loginfo("New start was set")
                if self.ready_to_plan():
                    self.replan()
            else:
                rospy.logwarn("New start is bad or no map available")
            self.is_working = False

    def new_map_callback(self, grid_map):
        if not self.is_working:
            self.is_working = True
            self.map = Map(grid_map)
            rospy.loginfo("New map was set")
            if self.ready_to_plan():
                self.replan()
            self.is_working = False

    def replan(self):
    	global k 
        rospy.loginfo("Planning was started")
        final_state = self.planner(self.map, self.moves, self.robot, self.start, self.goal, self.pose_publisher)

        if final_state is None:
            rospy.loginfo("No path found")
        else:
            # Restore and publish path
            rospy.loginfo("Restoring path from final state...")
            path = self.restore_path(final_state)
            self.path_publisher.publish(path)
            rospy.loginfo("Planning was finished...")
#Waypoint goals Test section
            self.start = final_state #test
            rospy.loginfo("final goal is the next start point") #test
            while True:
            	if State.sdistance(State.from_pose(dr_pose.pose),final_state)<0.07 and State.sangle(State.from_pose(dr_pose.pose),final_state)<0.07:
            		break
            if k == 1:
	        	posemsg.pose.position.x = 2
	        	posemsg.pose.position.y = 0
	        	posemsg.pose.position.z = 0.3
	        	posemsg.pose.orientation.x = 0
	        	posemsg.pose.orientation.y = 0
	        	posemsg.pose.orientation.z = 0
	        	posemsg.pose.orientation.w = 1
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 2:
	        	posemsg.pose.position.x = 1.5
	        	posemsg.pose.position.y = -1
	        	posemsg.pose.orientation.z = 0.924 #135 CCW wrt origin
	        	posemsg.pose.orientation.w = 0.383
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 3:
	        	posemsg.pose.position.x = 0
	        	posemsg.pose.position.y = 1
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 4:
	        	posemsg.pose.orientation.z = 1 #180 CCW wrt origin
	        	posemsg.pose.orientation.w = 0    
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 5:   
	        	posemsg.pose.position.x = -1
	        	posemsg.pose.position.y = 1 
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 6:   
	        	posemsg.pose.position.x = -1.75
	        	posemsg.pose.position.y = 1  
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 7:   
	        	posemsg.pose.orientation.z = 0.707 #270 CCW
	        	posemsg.pose.orientation.w = -0.707  
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 8:   
	        	posemsg.pose.position.x = -1.75
	        	posemsg.pose.position.y = 0.5
	        	posemsg.pose.orientation.z = 0 #0
	        	posemsg.pose.orientation.w = 1
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 9:   
	        	posemsg.pose.position.x = -0.8
	        	posemsg.pose.position.y = 0.5
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 10:   
	        	posemsg.pose.orientation.z = 0.707 #270 CCW
	        	posemsg.pose.orientation.w = -0.707 
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 11:   
	        	posemsg.pose.position.x = -0.8
	        	posemsg.pose.position.y = 0
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 12:   
	        	posemsg.pose.orientation.z = 1 #180 CCW wrt origin
	        	posemsg.pose.orientation.w = 0 
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 13:   
	        	posemsg.pose.position.x = -1.5
	        	posemsg.pose.position.y = 0
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 14:   
	        	posemsg.pose.position.x = -2.5
	        	posemsg.pose.position.y = 0
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 15:   
	        	posemsg.pose.orientation.z = 0.707 #90 CCW
	        	posemsg.pose.orientation.w = 0.707 
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 16:   
	        	posemsg.pose.position.x = -2.5
	        	posemsg.pose.position.y = 0.5    
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 17:   
	        	posemsg.pose.orientation.z = 1 #180 CCW wrt origin
	        	posemsg.pose.orientation.w = 0  
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 18:     
	        	posemsg.pose.position.x = -3.25
	        	posemsg.pose.position.y = 0.5
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 19:    
	        	posemsg.pose.position.x = -2.5
	        	posemsg.pose.position.y = 0.5
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 20:   
	        	posemsg.pose.orientation.z = 0.707 #270 CCW
	        	posemsg.pose.orientation.w = -0.707  
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 21:   
	        	posemsg.pose.position.x = -2.5
	        	posemsg.pose.position.y = 0     
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 22:   
	        	posemsg.pose.position.x = -2.5
	        	posemsg.pose.position.y = -1    
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 23:   
	        	posemsg.pose.orientation.z = 0 #0
	        	posemsg.pose.orientation.w = 1    
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 24:   
	        	posemsg.pose.position.x = -2
	        	posemsg.pose.position.y = -1  
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 25:   
	        	posemsg.pose.position.x = -1
	        	posemsg.pose.position.y = -0.5   
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 26:   
	        	posemsg.pose.position.x = -0.25
	        	posemsg.pose.position.y = -0.75   
	        	posemsg.pose.orientation.z = 0.383 #45 CCW wrt origin
	        	posemsg.pose.orientation.w = 0.924
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 27:   
	        	posemsg.pose.position.x = 0.5
	        	posemsg.pose.position.y = 0 
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            elif k == 28:   
	        	posemsg.pose.position.x = 1.5
	        	posemsg.pose.position.y = 1   
	        	self.goal = State.from_pose(posemsg.pose)
	        	rospy.loginfo("New goal was set")
	        	k += 1;
            else:  
	        	rospy.loginfo("Planning finished...")
	        	return


            self.replan()
       

    def restore_path(self, final_state):
        current_state = copy.copy(final_state)
        path = MarkerArray()
        pose_id = 0
        while True:
            pose_marker = current_state.to_marker(self.robot)
            pose_marker.id = pose_id
            path.markers.append(pose_marker)

            current_state = current_state.parent
            pose_id += 1

            if current_state is None:
                break
        return path


def main():
    rospy.init_node("trajectory_planner")
    planner = TrajectoryPlanner()
    rospy.spin()

main()
