#!/usr/bin/python2.7
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
import visualization_msgs
import copy
import planners.astar
import json
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from move import Move
from state import State
from robot import Robot
from map import Map

#Global variables
k = 1
g = 1
A1 = A2 = B1 = B2 = C1 = C2 = D1 = D2 = E1 = E2 = F1 = F2 = G1 = G2 = H1 = H2 = I1 = I2 = J1 = J2 = K1 = K2 = L1 = L2 = 1
dr_pose = None
pis = 1

#Define posestamped message for waypoints
posemsg = PoseStamped()
posemsg.header.frame_id = 'cf1/odom'

# Load world JSON
with open('/home/maverick/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/comp_without_signs.world.json') as f:
    world = json.load(f)

class TrajectoryPlanner:
    def __init__(self):
        global pis
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
        self.drone_subscriber = rospy.Subscriber("/cf1/pose", PoseStamped, self.pose_callback)
        #self.waypoint_subscriber = rospy.Subscriber("waypoint", PoseStamped, self.new_goal_callback)

        self.path_publisher = rospy.Publisher("trajectory", MarkerArray, queue_size=3)
        self.pose_publisher = rospy.Publisher("debug_pose", PoseStamped, queue_size=3)
        self.rotate = rospy.Publisher("cf1/cmd_position", Position, queue_size=3)

        # what will be there. A module goes into variable. Isn't it too much memory consumption. Maybe I should assign function replan() to this variable?
        self.planner = planners.astar.replan
        pis = self.gates(world)

    def pose_callback(self, d_pose):
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
        global k, pis, A1, A2, B1, B2, C1, C2, D1, D2, E1, E2, F1, F2, G1, G2, H1, H2, I1, I2, J1, J2, K1, K2, L1, L2
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
                if State.sdistance(State.from_pose(dr_pose.pose),final_state)<0.03 and State.sangle(State.from_pose(dr_pose.pose),final_state)<0.06:
                    #rotate = Position()
                    #rotate.header.stamp = rospy.Time.now()
                    #rotate.header.frame_id = 'cf1/odom'
                    #rotate.x = dr_pose.pose.position.x
                    #rotate.y = dr_pose.pose.position.y
                    #rotate.z = dr_pose.pose.position.z
                    #roll, pitch, yawt = euler_from_quaternion((dr_pose.pose.orientation.x,
                    #                          dr_pose.pose.orientation.y,
                    #                          dr_pose.pose.orientation.z,
                    #                          dr_pose.pose.orientation.w))
                    #for r in range(1,5):
                    #rotate.yaw = math.degrees(yawt) + 90
                    #rospy.loginfo("Oh man")
                    #self.rotate.publish(rotate)
                    #    rospy.sleep(5)
                    break
            if k < 2*pis:
                if k == 1:
                    self.goal = A1
                    k += 1;
                elif k == 2:
                    self.goal = A2
                    k += 1;
                elif k == 3:
                    self.goal = B1
                    k += 1;
                elif k == 4:
                    self.goal = B2
                    k += 1;
                elif k == 5:
                    self.goal = C1
                    k += 1;                
                elif k == 6:
                    self.goal = C2
                    k += 1;
                elif k == 7:
                    self.goal = D1
                    k += 1;
                elif k == 8:
                    self.goal = D2
                    k += 1;
                elif k == 9:
                    self.goal = E1
                    k += 1;
                elif k == 10:
                    self.goal = E2
                    k += 1;
                elif k == 11:
                    self.goal = F1
                    k += 1;
                elif k == 12:
                    self.goal = F2
                    k += 1;
                elif k == 13:
                    self.goal = G1
                    k += 1;
                elif k == 14:
                    self.goal = G2
                    k += 1;
                elif k == 15:
                    self.goal = H1
                    k += 1;
                elif k == 16:
                    self.goal = H2
                    k += 1;
                elif k == 17:
                    self.goal = I1
                    k += 1;
                elif k == 18:
                    self.goal = I2
                    k += 1;
                elif k == 19:
                    self.goal = J1
                    k += 1;
                elif k == 20:
                    self.goal = J2
                    k += 1;
                elif k == 21:
                    self.goal = K1
                    k += 1;
                elif k == 22:
                    self.goal = K2
                    k += 1;
                elif k == 23:
                    self.goal = L1
                    k += 1;
                elif k == 24:
                    self.goal = L2
                    k += 1;

                else:  
                    rospy.loginfo("Planning finished...")
                    return

                self.replan()

    def gates(self,world):
        global g, A1, A2, B1, B2, C1, C2, D1, D2, E1, E2, F1, F2, G1, G2, H1, H2, I1, I2, J1, J2, K1, K2, L1, L2
        p = 0
        for id in world['gates']:
            h = world['gates'][p]['heading'] - 90
            a = world['gates'][p]['position'][0]
            b = world['gates'][p]['position'][1]

            (xl,yl) = (a - 0.3*math.sin(math.radians(h)),b + 0.3*math.cos(math.radians(h)))
            (xr,yr) = (a + 0.3*math.sin(math.radians(h)),b - 0.3*math.cos(math.radians(h)))
            #while True:
             #   if self.waypoint_pub.get_num_connections()>0:
              #      break
            if g == 1:
                A1 = State.from_pose(self.way_to_pose(xl,yl,h))
                A2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 2:
                B1 = State.from_pose(self.way_to_pose(xl,yl,h))
                B2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1;
            elif g == 3:
                C1 = State.from_pose(self.way_to_pose(xl,yl,h))
                C2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 4:
                D1 = State.from_pose(self.way_to_pose(xl,yl,h))
                D2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 5:
                E1 = State.from_pose(self.way_to_pose(xl,yl,h))
                E2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 6:
                F1 = State.from_pose(self.way_to_pose(xl,yl,h))
                F2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 7:
                G1 = State.from_pose(self.way_to_pose(xl,yl,h))
                G2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 8:
                H1 = State.from_pose(self.way_to_pose(xl,yl,h))
                H2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 9:
                I1 = State.from_pose(self.way_to_pose(xl,yl,h))
                I2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 10:
                J1 = State.from_pose(self.way_to_pose(xl,yl,h))
                J2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 11:
                K1 = State.from_pose(self.way_to_pose(xl,yl,h))
                K2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1
            elif g == 12:
                L1 = State.from_pose(self.way_to_pose(xl,yl,h))
                L2 = State.from_pose(self.way_to_pose(xr,yr,h))
                g += 1

            p += 1
        return p

    def dist_poses(self,p,q):
        dx = p.pose.position.x - q.pose.position.x
        dy = p.pose.position.y - q.pose.position.y
        dz = p.pose.position.z - q.pose.position.z

        return(dx**2 + dy**2 + dz**2)**0.5

    def way_to_pose(self, x, y, h):
        way = PoseStamped()
        way.header.stamp = rospy.Time.now()
        way.header.frame_id = 'cf1/odom'
        way.pose.position.x = x
        way.pose.position.y = y
        way.pose.position.z = 0.3

        (way.pose.orientation.x, 
        way.pose.orientation.y, 
        way.pose.orientation.z, 
        way.pose.orientation.w) = quaternion_from_euler(math.radians(0),math.radians(0),math.radians(h))

        return way.pose
       

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
