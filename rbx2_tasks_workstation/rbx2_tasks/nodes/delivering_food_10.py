#!/usr/bin/env python

import rospy
import smach
import actionlib
from smach import State, StateMachine, Concurrence
from smach_ros import SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from visualization_msgs.msg import Marker
from ar_track_alvar.msg import AlvarMarkers

import easygui
import datetime
from collections import OrderedDict
from tf.transformations import quaternion_from_euler
from math import radians, pi
import random

import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

from custom_state_class.computer_vision_beep import ComputerVision
from custom_state_class.search_table_beep import SearchTable
from custom_state_class.do_stuffs_beep import DoStuffs
from custom_state_class.rotate360 import Rotate360

from std_msgs.msg import String

class Welcome(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded'])
        self.soundstring_publisher = rospy.Publisher('sound_message', String, queue_size=10)

    def execute(self, userdata):
        rospy.sleep(1)
        rospy.loginfo("Hello, I am kura-kurabot")
        rospy.loginfo("I will help you to deliver food to your customers")

        self.soundstring_publisher.publish("welcome")
        rospy.sleep(8)

        return 'succeeded'


class MyMainClass():
    def __init__(self):
        rospy.init_node('deliver_food', anonymous=False)
        self.initialize_destination()

        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0

        nav_states = {}
        
        for room in self.room_locations.iterkeys():
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = self.room_locations[room]
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb, 
                                                exec_timeout=rospy.Duration(60.0),
                                                server_wait_timeout=rospy.Duration(10.0))
            nav_states[room] = move_base_state
            rospy.loginfo(room + " -> [" + str(round(self.room_locations[room].position.x,2)) + ", " + str(round(self.room_locations[room].position.y,2)) + "]" )

        sm_rotate_search = Concurrence(outcomes=['find', 'not_find'],
                                        default_outcome='not_find',
                                        child_termination_cb=self.concurrence_child_termination_callback,
                                        outcome_cb=self.concurrence_outcome_callback)

        with sm_rotate_search:
            Concurrence.add('ROTATE', Rotate360(0.4, 2*pi) )
            Concurrence.add('SEARCH', SearchTable() )        

        sm_table1 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table1:
            StateMachine.add('GOTO_TABLE1', nav_states['table1'], transitions={'succeeded':'ROTATE_SEARCH','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('ROTATE_SEARCH', sm_rotate_search, transitions={'find':'DO_STUFFS','not_find':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs(5), transitions={'succeeded':'GOTO_KITCHEN','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'','aborted':'','preempted':''})

        sm_table2 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table2:
            StateMachine.add('GOTO_TABLE2', nav_states['table2'], transitions={'succeeded':'ROTATE_SEARCH','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('ROTATE_SEARCH', sm_rotate_search, transitions={'find':'DO_STUFFS','not_find':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs(5), transitions={'succeeded':'GOTO_KITCHEN','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'','aborted':'','preempted':''})

        sm_table3 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table3:
            StateMachine.add('GOTO_TABLE3', nav_states['table3'], transitions={'succeeded':'ROTATE_SEARCH',
                                                                               'aborted':'GOTO_CHECKPOINT_GO_2',
                                                                               'preempted':'GOTO_CHECKPOINT_GO_2'})
            StateMachine.add('ROTATE_SEARCH', sm_rotate_search, transitions={'find':'DO_STUFFS',
                                                                             'not_find':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs(5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'GOTO_CHECKPOINT_BACK_1'})

            # if something wrong when we tried to go to the table

            StateMachine.add('GOTO_CHECKPOINT_GO_2', nav_states['checkpoint2'], transitions={'succeeded':'GOTO_TABLE3_2',
            																	   		  	 'aborted':'GOTO_CHECKPOINT_GO_1'} )

            StateMachine.add('GOTO_CHECKPOINT_GO_1', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_CHECKPOINT_GO_2a',
            																	     	  	 'aborted':'GOTO_KITCHEN'} )

            StateMachine.add('GOTO_CHECKPOINT_GO_2a', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_TABLE3_2',
            																				  'aborted':'GOTO_KITCHEN'} )

            StateMachine.add('GOTO_TABLE3_2', nav_states['table3'], transitions={'succeeded':'ROTATE_SEARCH',
            																	 'aborted':'CLEARING_NOISE_GO'})

            StateMachine.add('CLEARING_NOISE_GO', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_TABLE3_3',
            																		 'not_full_rotate':'GOTO_KITCHEN'})

            StateMachine.add('GOTO_TABLE3_3', nav_states['table3'], transitions={'succeeded':'ROTATE_SEARCH',
            																	 'aborted':'GOTO_KITCHEN'})

            # if something wrong when we tried to go back to kitchen

            StateMachine.add('GOTO_CHECKPOINT_BACK_1', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_KITCHEN_2',
            																			   	   'aborted':'GOTO_CHECKPOINT_BACK_2'})

            StateMachine.add('GOTO_CHECKPOINT_BACK_2', nav_states['checkpoint2'], transitions={'succeeded':'GOTO_CHECKPOINT_BACK_1a',
            																			   	   'aborted':'CLEARING_NOISE_BACK_a'})

            StateMachine.add('CLEARING_NOISE_BACK_a', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_CHECKPOINT_BACK_2',
            																	 		 'not_full_rotate':'GOTO_CHECKPOINT_BACK_2'})

            StateMachine.add('GOTO_CHECKPOINT_BACK_1a', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_KITCHEN_2',
            																			   	    'aborted':'CLEARING_NOISE_BACK_b'})

            StateMachine.add('CLEARING_NOISE_BACK_b', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_CHECKPOINT_BACK_1a',
            																	 		 'not_full_rotate':'GOTO_CHECKPOINT_BACK_1a'})

            StateMachine.add('GOTO_KITCHEN_2', nav_states['kitchen'], transitions={'succeeded':'',
            																	   'aborted':'CLEARING_NOISE_BACK_c'})

            StateMachine.add('CLEARING_NOISE_BACK_c', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_KITCHEN_2',
            																	 		 'not_full_rotate':'GOTO_KITCHEN_2'})

        sm_table4 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table4:
            StateMachine.add('GOTO_TABLE4', nav_states['table4'], transitions={'succeeded':'ROTATE_SEARCH',
                                                                               'aborted':'GOTO_CHECKPOINT_GO_2',
                                                                               'preempted':'GOTO_CHECKPOINT_GO_2'})
            StateMachine.add('ROTATE_SEARCH', sm_rotate_search, transitions={'find':'DO_STUFFS',
                                                                             'not_find':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs(5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'GOTO_CHECKPOINT_BACK_1',
                                                                                 'preempted':'GOTO_CHECKPOINT_BACK_1'})

            # if something wrong when we tried to go to the table

            StateMachine.add('GOTO_CHECKPOINT_GO_2', nav_states['checkpoint2'], transitions={'succeeded':'GOTO_TABLE4_2',
            																	   		  	 'aborted':'GOTO_CHECKPOINT_GO_1'} )

            StateMachine.add('GOTO_CHECKPOINT_GO_1', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_CHECKPOINT_GO_2a',
            																	     	  	 'aborted':'GOTO_KITCHEN'} )

            StateMachine.add('GOTO_CHECKPOINT_GO_2a', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_TABLE4_2',
            																				  'aborted':'GOTO_KITCHEN'} )

            StateMachine.add('GOTO_TABLE4_2', nav_states['table4'], transitions={'succeeded':'ROTATE_SEARCH',
            																	 'aborted':'CLEARING_NOISE_GO'})

            StateMachine.add('CLEARING_NOISE_GO', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_TABLE4_3',
            																		 'not_full_rotate':'GOTO_KITCHEN'})

            StateMachine.add('GOTO_TABLE4_3', nav_states['table4'], transitions={'succeeded':'ROTATE_SEARCH',
            																	 'aborted':'GOTO_KITCHEN'})

            # if something wrong when we tried to go back to kitchen

            StateMachine.add('GOTO_CHECKPOINT_BACK_1', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_KITCHEN_2',
            																			   	   'aborted':'GOTO_CHECKPOINT_BACK_2'})

            StateMachine.add('GOTO_CHECKPOINT_BACK_2', nav_states['checkpoint2'], transitions={'succeeded':'GOTO_CHECKPOINT_BACK_1a',
            																			   	   'aborted':'CLEARING_NOISE_BACK_a'})

            StateMachine.add('CLEARING_NOISE_BACK_a', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_CHECKPOINT_BACK_2',
            																	 		 'not_full_rotate':'GOTO_CHECKPOINT_BACK_2'})

            StateMachine.add('GOTO_CHECKPOINT_BACK_1a', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_KITCHEN_2',
            																			   	    'aborted':'CLEARING_NOISE_BACK_b'})

            StateMachine.add('CLEARING_NOISE_BACK_b', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_CHECKPOINT_BACK_1a',
            																	 		 'not_full_rotate':'GOTO_CHECKPOINT_BACK_1a'})

            StateMachine.add('GOTO_KITCHEN_2', nav_states['kitchen'], transitions={'succeeded':'',
            																	   'aborted':'CLEARING_NOISE_BACK_c'})

            StateMachine.add('CLEARING_NOISE_BACK_c', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_KITCHEN_2',
            																	 		 'not_full_rotate':'GOTO_KITCHEN_2'})


        sm_table5 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table5:
            StateMachine.add('GOTO_TABLE5', nav_states['table5'], transitions={'succeeded':'ROTATE_SEARCH','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('ROTATE_SEARCH', sm_rotate_search, transitions={'find':'DO_STUFFS','not_find':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs(5), transitions={'succeeded':'GOTO_KITCHEN','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'','aborted':'','preempted':''})

        sm_table6 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table6:
            StateMachine.add('GOTO_TABLE6', nav_states['table6'], transitions={'succeeded':'ROTATE_SEARCH',
                                                                               'aborted':'GOTO_CHECKPOINT_GO_2',
                                                                               'preempted':'GOTO_CHECKPOINT_GO_2'})
            StateMachine.add('ROTATE_SEARCH', sm_rotate_search, transitions={'find':'DO_STUFFS',
                                                                             'not_find':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs(5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'GOTO_CHECKPOINT_BACK_1',
                                                                                 'preempted':'GOTO_CHECKPOINT_BACK_1'})

            # if something wrong when we tried to go to the table

            StateMachine.add('GOTO_CHECKPOINT_GO_2', nav_states['checkpoint2'], transitions={'succeeded':'GOTO_TABLE6_2',
            																	   		  	 'aborted':'GOTO_CHECKPOINT_GO_1'} )

            StateMachine.add('GOTO_CHECKPOINT_GO_1', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_CHECKPOINT_GO_2a',
            																	     	  	 'aborted':'GOTO_KITCHEN'} )

            StateMachine.add('GOTO_CHECKPOINT_GO_2a', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_TABLE6_2',
            																				  'aborted':'GOTO_KITCHEN'} )

            StateMachine.add('GOTO_TABLE6_2', nav_states['table6'], transitions={'succeeded':'ROTATE_SEARCH',
            																	 'aborted':'CLEARING_NOISE_GO'})

            StateMachine.add('CLEARING_NOISE_GO', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_TABLE6_3',
            																		 'not_full_rotate':'GOTO_KITCHEN'})

            StateMachine.add('GOTO_TABLE6_3', nav_states['table6'], transitions={'succeeded':'ROTATE_SEARCH',
            																	 'aborted':'GOTO_KITCHEN'})

            # if something wrong when we tried to go back to kitchen

            StateMachine.add('GOTO_CHECKPOINT_BACK_1', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_KITCHEN_2',
            																			   	   'aborted':'GOTO_CHECKPOINT_BACK_2'})

            StateMachine.add('GOTO_CHECKPOINT_BACK_2', nav_states['checkpoint2'], transitions={'succeeded':'GOTO_CHECKPOINT_BACK_1a',
            																			   	   'aborted':'CLEARING_NOISE_BACK_a'})

            StateMachine.add('CLEARING_NOISE_BACK_a', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_CHECKPOINT_BACK_2',
            																	 		 'not_full_rotate':'GOTO_CHECKPOINT_BACK_2'})

            StateMachine.add('GOTO_CHECKPOINT_BACK_1a', nav_states['checkpoint1'], transitions={'succeeded':'GOTO_KITCHEN_2',
            																			   	    'aborted':'CLEARING_NOISE_BACK_b'})

            StateMachine.add('CLEARING_NOISE_BACK_b', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_CHECKPOINT_BACK_1a',
            																	 		 'not_full_rotate':'GOTO_CHECKPOINT_BACK_1a'})

            StateMachine.add('GOTO_KITCHEN_2', nav_states['kitchen'], transitions={'succeeded':'',
            																	   'aborted':'CLEARING_NOISE_BACK_c'})

            StateMachine.add('CLEARING_NOISE_BACK_c', Rotate360(0.8, 2*pi), transitions={'full_rotate':'GOTO_KITCHEN_2',
            																	 		 'not_full_rotate':'GOTO_KITCHEN_2'})
        # let's initialize the overall state machine
        sm_deliverfood = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with sm_deliverfood:
            StateMachine.add('STARTING_TASK', Welcome(), transitions={'succeeded':'GOTO_KITCHEN'})
            StateMachine.add('COMPUTER_VISION_TASK', ComputerVision(), transitions={'detect1':'TABLE_ONE_TASK',
                                                                                  'detect2':'TABLE_TWO_TASK',
                                                                                  'detect3':'TABLE_THREE_TASK',
                                                                                  'detect4':'TABLE_FOUR_TASK',
                                                                                  'detect5':'TABLE_FIVE_TASK',
                                                                                  'detect6':'TABLE_SIX_TASK',
                                                                                  'preempted':''})
            StateMachine.add('TABLE_ONE_TASK',sm_table1, transitions={'succeeded':'COMPUTER_VISION_TASK','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('TABLE_TWO_TASK',sm_table2, transitions={'succeeded':'COMPUTER_VISION_TASK','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('TABLE_THREE_TASK',sm_table3, transitions={'succeeded':'COMPUTER_VISION_TASK','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('TABLE_FOUR_TASK',sm_table4, transitions={'succeeded':'COMPUTER_VISION_TASK','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('TABLE_FIVE_TASK',sm_table5, transitions={'succeeded':'COMPUTER_VISION_TASK','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})
            StateMachine.add('TABLE_SIX_TASK',sm_table6, transitions={'succeeded':'COMPUTER_VISION_TASK','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})

            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'COMPUTER_VISION_TASK','aborted':'GOTO_KITCHEN','preempted':'GOTO_KITCHEN'})

        # Create and start the SMACH introspection server
        intro_server = IntrospectionServer('deliver_food', sm_deliverfood, '/SM_ROOT')
        intro_server.start()
        
        # Execute the state machine
        sm_outcome = sm_deliverfood.execute()
        rospy.on_shutdown(self.shutdown)

    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.n_succeeded += 1
        elif status == actionlib.GoalStatus.ABORTED:
            self.n_aborted += 1
        elif status == actionlib.GoalStatus.PREEMPTED:
            self.n_preempted += 1


    def concurrence_child_termination_callback(self, outcome_map):
        if outcome_map['SEARCH'] == 'detect':
          return True

        elif outcome_map['ROTATE'] == 'full_rotate':
          return True

        else:
          return False

    def concurrence_outcome_callback(self, outcome_map):
        if outcome_map['SEARCH'] == 'detect':
          return 'find'

        elif outcome_map['ROTATE'] == 'full_rotate':
          return 'not_find'

        # lazy to think, just put this
        else:
          return 'not_find'

    def initialize_destination(self):
        self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) # seconds

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base action server")

        # Orientation list
        quaternions = list()

        euler_angles = (0, -pi/2, pi, 0, pi/2, -pi/2, -3*pi/4, pi/2, pi/2)

        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Position list
        points = list()
        points.append(Point(0, 0, 0))
        points.append(Point(0, -1.55497133732, 0))
        points.append(Point(-2.21499538422, -1.55497133732, 0))
        points.append(Point(-1.3093624115, 0.198241680861, 0))
        points.append(Point(-3.51539182663, 0, 0))
        points.append(Point(-0.926210045815, -2.36771917343, 0))
        points.append(Point(-3.55022335052, -1.96715021133, 0))
        points.append(Point(-1.13520205021, -1.3924216032, 0))
        points.append(Point(-2.56331586838, -0.289407044649, 0))
        
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
        self.waypoints.append( Pose(points[7], quaternions[7]) )
        self.waypoints.append( Pose(points[8], quaternions[8]) )

        # Create a mapping of room names to waypoint locations
        room_locations = (('kitchen', self.waypoints[0]),
                          ('table1', self.waypoints[1]),
                          ('table2', self.waypoints[2]),
                          ('table3', self.waypoints[3]),
                          ('table4', self.waypoints[4]),
                          ('table5', self.waypoints[5]),
                          ('table6', self.waypoints[6]),
                          ('checkpoint1', self.waypoints[7]),
                          ('checkpoint2', self.waypoints[8]))
        
        # Store the mapping as an ordered dictionary so we can visit the rooms in sequence
        self.room_locations = OrderedDict(room_locations)

       	#initialize position
        initial_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=20)
        rospy.sleep(3)
        posewithcovariance_msg = PoseWithCovarianceStamped()
        posewithcovariance_msg.pose.pose= self.room_locations['checkpoint1']
        initial_publisher.publish(posewithcovariance_msg)  
            
        # Initialize a marker for the docking station for RViz
        self.init_waypoint_markers()
        self.init_docking_station_marker()

        self.init_waypoint_markers()     
        for waypoint in self.waypoints:           
            p = Point()
            p = waypoint.position
            self.waypoint_markers.points.append(p)
            
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
    
    def init_waypoint_markers(self):# Set up our waypoint markers
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
        self.docking_station_marker.pose = self.waypoints[0]
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.sm_deliverfood.request_preempt()  
        self.cmd_vel_pub.publish(Twist())        
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MyMainClass()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")