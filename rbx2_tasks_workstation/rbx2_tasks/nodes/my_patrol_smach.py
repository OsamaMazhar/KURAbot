#!/usr/bin/env python

""" patrol_smach.py - Version 1.0 2013-04-12

    Control a robot to patrol a square area using SMACH

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import smach
import actionlib
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class Patrol():
    def __init__(self):
        rospy.init_node('patrol_smach', anonymous=False)
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize a number of parameters and variables
        self.setup_task_environment()
        #self.setup_initial_pose()

        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0

        # A list to hold then navigation waypoints
        nav_states = list()
        
        # Turn the waypoints into SMACH states
        for waypoint in self.waypoints:           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = waypoint
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(30.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            nav_states.append(move_base_state)
        
        # Initialize the patrol state machine
        self.sm_patrol = StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Add the states to the state machine with the appropriate transitions
        with self.sm_patrol:            
            StateMachine.add('NAV_STATE_0', nav_states[0], transitions={'succeeded':'NAV_STATE_1','aborted':'NAV_STATE_1','preempted':'NAV_STATE_1'})
            StateMachine.add('NAV_STATE_1', nav_states[1], transitions={'succeeded':'NAV_STATE_2','aborted':'NAV_STATE_2','preempted':'NAV_STATE_2'})
            StateMachine.add('NAV_STATE_2', nav_states[2], transitions={'succeeded':'NAV_STATE_3','aborted':'NAV_STATE_3','preempted':'NAV_STATE_3'})
            StateMachine.add('NAV_STATE_3', nav_states[3], transitions={'succeeded':'NAV_STATE_4','aborted':'NAV_STATE_4','preempted':'NAV_STATE_4'})
            StateMachine.add('NAV_STATE_4', nav_states[4], transitions={'succeeded':'NAV_STATE_5','aborted':'NAV_STATE_5','preempted':'NAV_STATE_4'})
            StateMachine.add('NAV_STATE_5', nav_states[5], transitions={'succeeded':'NAV_STATE_6','aborted':'NAV_STATE_6','preempted':'NAV_STATE_4'})
            StateMachine.add('NAV_STATE_6', nav_states[6], transitions={'succeeded':'','aborted':'','preempted':''})
            
        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('patrol', self.sm_patrol, '/SM_ROOT')
        intro_server.start()
        
        sm_outcome = self.sm_patrol.execute()
        rospy.loginfo("FINISHED PATROL LOOP: " + str(self.patrol_count))
        
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
                
        intro_server.stop()


    def setup_initial_pose(self):
        # rostopic pub /initialpose geometry_msgs/PoseWithCovarianceSmped '{ header: { frame_id: "map" }, pose: { pose: { position: { x: 2.6731004715, y: 2.44490814209, z: 0 }, orientation: { x: 0 , y: 0, z: 0, w: 1 } } } }'

        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, latch=True)

        mypose = PoseWithCovarianceStamped()
        mypose.header.frame_id = 'map'
        mypose.pose.pose = self.waypoints[1]

        self.initial_pose_pub.publish(mypose)

    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.n_succeeded += 1
        elif status == actionlib.GoalStatus.ABORTED:
            self.n_aborted += 1
        elif status == actionlib.GoalStatus.PREEMPTED:
            self.n_preempted += 1

        try:
            rospy.loginfo("n_succeeded "+ str(self.n_succeeded))
            rospy.loginfo("n_aborted "+ str(self.n_aborted))
            rospy.loginfo("n_preempted "+ str(self.n_preempted))
            rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
        except:
            pass

    def setup_task_environment(self):
        # How big is the square we want the robot to patrol?
        self.square_size = rospy.get_param("~square_size", 1.0) # meters
        
        # Set the low battery threshold (between 0 and 100)
        self.low_battery_threshold = rospy.get_param('~low_battery_threshold', 50)
        
        # How many times should we execute the patrol loop
        self.n_patrols = rospy.get_param("~n_patrols", 2) # meters
        
        # How long do we have to get to each waypoint?
        self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) #seconds
        
        # Initialize the patrol counter
        self.patrol_count = 0
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait up to 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))    
        
        rospy.loginfo("Connected to move_base action server")
        
        # Orientation list
        quaternions = list()
        quaternions.append(Quaternion(0.0, 0.0, 0.0, 1.0))
        quaternions.append(Quaternion(0.0, 0.0, 1.0, 6.12303176911))
        quaternions.append(Quaternion(0.0, 0.0, 0.707106781187, 0.707106781187))
        quaternions.append(Quaternion(0.0, 0.0, 1.0, -0.0034963161226))
        quaternions.append(Quaternion(0.0, 0.0, -0.707106781187, 0.707106781187))
        quaternions.append(Quaternion(0.0, 0.0, 0.707106781187, 0.707106781187))
        quaternions.append(Quaternion(0.0, 0.0, 0.707106781187, 0.707106781187))

        # Position list
        points = list()
        points.append(Point(2.6731004715, 2.44490814209, 0))
        points.append(Point(2.6731004715, 0.81360769272, 0))
        points.append(Point(0.4786823391, 0.81360769272, 0))
        points.append(Point(1.6281396152, 2.44490814209, 0))
        points.append(Point(-0.6823855638, 2.44490814209, 0))
        points.append(Point(1.7326359748, -0.06299890577, 0))
        points.append(Point(-0.7520495653, 0.01827591657, 0))
        
        # Create a list to hold the waypoint poses
        self.waypoints = list()
                
        # Append each of the four waypoints to the list.  Each waypoint
        # is a pose consisting of a position and orientation in the map frame.
        self.waypoints.append( Pose(points[0], quaternions[0]) )
        self.waypoints.append( Pose(points[1], quaternions[1]) )
        self.waypoints.append( Pose(points[2], quaternions[2]) )
        self.waypoints.append( Pose(points[3], quaternions[3]) )
        self.waypoints.append( Pose(points[4], quaternions[4]) )
        self.waypoints.append( Pose(points[5], quaternions[5]) )
        self.waypoints.append( Pose(points[6], quaternions[6]) )
                
        # Where is the docking station?
        # let's consider docking station is our kitchen
        self.docking_station_pose = (Pose(points[0], quaternions[0]))            
        
        # Initialize markers for the waypoints for RViz
        self.init_waypoint_markers()
        
        # Set a visualization marker at each waypoint        
        for waypoint in self.waypoints[1:]:           
            p = Point()
            p = waypoint.position
            self.waypoint_markers.points.append(p)
            
        # Initialize a marker for the docking station for RViz
        self.init_docking_station_marker()
            
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        rospy.loginfo("Starting Tasks")
        
        # Publish the waypoint markers
        self.marker_pub.publish(self.waypoint_markers)
        rospy.sleep(1)
        self.marker_pub.publish(self.waypoint_markers) # <- this is weird
        
        # Publish the docking station marker
        self.docking_station_marker_pub.publish(self.docking_station_marker)
        rospy.sleep(1)

    def init_waypoint_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker)
        
        # Initialize the marker points list.
        self.waypoint_markers = Marker()
        self.waypoint_markers.ns = marker_ns
        self.waypoint_markers.id = marker_id
        self.waypoint_markers.type = Marker.CUBE_LIST
        self.waypoint_markers.action = Marker.ADD
        self.waypoint_markers.lifetime = rospy.Duration(marker_lifetime)
        self.waypoint_markers.scale.x = marker_scale
        self.waypoint_markers.scale.y = marker_scale
        self.waypoint_markers.color.r = marker_color['r']
        self.waypoint_markers.color.g = marker_color['g']
        self.waypoint_markers.color.b = marker_color['b']
        self.waypoint_markers.color.a = marker_color['a']
        
        self.waypoint_markers.header.frame_id = 'odom'
        self.waypoint_markers.header.stamp = rospy.Time.now()
        self.waypoint_markers.points = list()

    def init_docking_station_marker(self):
        # Define a marker for the charging station
        marker_scale = 0.3
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 0.7, 'g': 0.7, 'b': 0.0, 'a': 1.0}
        
        self.docking_station_marker_pub = rospy.Publisher('docking_station_marker', Marker)
        
        self.docking_station_marker = Marker()
        self.docking_station_marker.ns = marker_ns
        self.docking_station_marker.id = marker_id
        self.docking_station_marker.type = Marker.CYLINDER
        self.docking_station_marker.action = Marker.ADD
        self.docking_station_marker.lifetime = rospy.Duration(marker_lifetime)
        self.docking_station_marker.scale.x = marker_scale
        self.docking_station_marker.scale.y = marker_scale
        self.docking_station_marker.scale.z = 0.02
        self.docking_station_marker.color.r = marker_color['r']
        self.docking_station_marker.color.g = marker_color['g']
        self.docking_station_marker.color.b = marker_color['b']
        self.docking_station_marker.color.a = marker_color['a']
        
        self.docking_station_marker.header.frame_id = 'odom'
        self.docking_station_marker.header.stamp = rospy.Time.now()
        self.docking_station_marker.pose = self.docking_station_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        #self.sm_patrol.request_preempt()
        
        #self.cmd_vel_pub.publish(Twist())
        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
