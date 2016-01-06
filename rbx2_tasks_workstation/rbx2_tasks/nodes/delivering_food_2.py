#!/usr/bin/env python

import rospy
import smach
import actionlib
from smach import State, StateMachine
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

'''
class ComputerVision(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect1','detect2','detect3','detect4','detect5','detect6'])
        self.taskname="doing the computer vision task"

    def execute(self, userdata):
        rospy.loginfo("detecing number...")
        rospy.sleep(5)
        table = random.randint(1,6)
        rospy.loginfo("number detected! it is: " + str(table) )

        if (table==1):
            return 'detect1'
        elif (table==2):
            return 'detect2'
        elif (table==3):
            return 'detect3'
        elif (table==4):
            return 'detect4'
        elif (table==5):
            return 'detect4'
        else:
            return 'detect4'
'''

class ComputerVision(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect1','detect2','detect3','detect4','detect5','detect6'])
        self.taskname="doing the computer vision task"

        # initialize a subscriber of ar_pose_marker
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.my_listener)
        self.tag_ids = [0,1,2,3,4,5,6]

        self.marker_table_dictionary = {1:'detect1', 2:'detect2', 3:'detect3', 4:'detect4', 5:'detect5', 6:'detect6'}
        self.n_markers = 0
        self.my_outcome = self.marker_table_dictionary[1]
        self.my_tagid = 0

    def execute(self, userdata):
        # while there is no marker, hold the execution of execute function
        while(self.n_markers==0):
            rospy.loginfo("waiting for delivery command...")
            rospy.sleep(1)

       	# right after there is marker return an outcome
        rospy.loginfo("yeay, let's deliver food to Table" + str(self.my_tagid) + "!")
       	return self.my_outcome

    def my_listener(self, msg):

        # get the number of markers
        self.n_markers = len(msg.markers)

        # if there is no marker stop the listener
        # at least we already have the number of markers
        if self.n_markers == 0:
            return

        # assuming there are only 1 tag at once 
        for tag in msg.markers:

            # Skip any tags that are not in our list
            if self.tag_ids is not None and not tag.id in self.tag_ids:
                continue

            self.my_tagid   = tag.id
            self.my_outcome = self.marker_table_dictionary[tag.id]

class DoStuffs(State):
    def __init__(self, message, timer):
        State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.message=message
        self.timer=timer

    def execute(self, userdata):
        rospy.loginfo("waiting for custumer to take the food | " + self.message)
        counter = 0
        while (counter<self.timer):
            counter+=1
            rospy.loginfo(str(counter))
            rospy.sleep(1)
        rospy.loginfo("waiting is over, done!")
        return 'succeeded'

class MyMainClass():
    def __init__(self):
        rospy.init_node('deliver_food', anonymous=False)
        self.initialize_destination()
        self.lalala = 100

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

        sm_table1 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table1:
            StateMachine.add('GOTO_TABLE1', nav_states['table1'], transitions={'succeeded':'DO_STUFFS',
                                                                               'aborted':'GOTO_KITCHEN',
                                                                               'preempted':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs("testing",5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'',
                                                                                 'preempted':''})

        sm_table2 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table2:
            StateMachine.add('GOTO_TABLE2', nav_states['table2'], transitions={'succeeded':'DO_STUFFS',
                                                                               'aborted':'GOTO_KITCHEN',
                                                                               'preempted':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs("testing",5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'',
                                                                                 'preempted':''})

        sm_table3 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table3:
            StateMachine.add('GOTO_TABLE3', nav_states['table3'], transitions={'succeeded':'DO_STUFFS',
                                                                               'aborted':'GOTO_KITCHEN',
                                                                               'preempted':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs("testing",5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'',
                                                                                 'preempted':''})

        sm_table4 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table4:
            StateMachine.add('GOTO_TABLE4', nav_states['table4'], transitions={'succeeded':'DO_STUFFS',
                                                                               'aborted':'GOTO_KITCHEN',
                                                                               'preempted':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs("testing",5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'',
                                                                                 'preempted':''})

        sm_table5 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table5:
            StateMachine.add('GOTO_TABLE5', nav_states['table5'], transitions={'succeeded':'DO_STUFFS',
                                                                               'aborted':'GOTO_KITCHEN', 
                                                                               'preempted':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs("testing",5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'',
                                                                                 'preempted':''})

        sm_table6 = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_table6:
            StateMachine.add('GOTO_TABLE6', nav_states['table6'], transitions={'succeeded':'DO_STUFFS',
                                                                               'aborted':'GOTO_KITCHEN',
                                                                               'preempted':'GOTO_KITCHEN'})
            StateMachine.add('DO_STUFFS', DoStuffs("testing",5), transitions={'succeeded':'GOTO_KITCHEN',
                                                                              'aborted':'GOTO_KITCHEN',
                                                                              'preempted':'GOTO_KITCHEN'})
            StateMachine.add('GOTO_KITCHEN', nav_states['kitchen'], transitions={'succeeded':'',
                                                                                 'aborted':'',
                                                                                 'preempted':''})

        # let's initialize the overall state machine
        sm_deliverfood = StateMachine(outcomes=['succeeded','aborted','preempted'])

        with sm_deliverfood:
            StateMachine.add('COMPUTER_VISION_TASK', ComputerVision(), transitions={'detect1':'TABLE_ONE_TASK',
                                                                                  'detect2':'TABLE_TWO_TASK',
                                                                                  'detect3':'TABLE_THREE_TASK',
                                                                                  'detect4':'TABLE_FOUR_TASK',
                                                                                  'detect5':'TABLE_FIVE_TASK',
                                                                                  'detect6':'TABLE_SIX_TASK'})
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

        try:
            rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
        except:
            pass

    def initialize_destination(self):
        self.move_base_timeout = rospy.get_param("~move_base_timeout", 10) # seconds

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base action server")
        
        # Orientation list
        quaternions = list()
        quaternions.append(Quaternion(0.0, 0.0, 0.0, 1.0))
        quaternions.append(Quaternion(0.0, 0.0, 1.0, 0))
        quaternions.append(Quaternion(0.0, 0.0, 0.707106781187, 0.707106781187))
        quaternions.append(Quaternion(0.0, 0.0, 1.0, 0))
        quaternions.append(Quaternion(0.0, 0.0, -0.707106781187, 0.707106781187))
        quaternions.append(Quaternion(0.0, 0.0, 0.707106781187, 0.707106781187))
        quaternions.append(Quaternion(0.0, 0.0, 0.707106781187, 0.707106781187))

        # Position list
        points = list()
        points.append(Point(0, 0, 0))
        points.append(Point(0, -1.55497133732, 0))
        points.append(Point(-2.21499538422, -1.55497133732, 0))
        points.append(Point(-1.3093624115, 0.198241680861, 0))
        points.append(Point(-3.51539182663, 0, 0))
        points.append(Point(-0.926210045815, -2.36771917343, 0))
        points.append(Point(-3.46894884109, -2.2051692009, 0))
        
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

        # Create a mapping of room names to waypoint locations
        room_locations = (('kitchen', self.waypoints[0]),
                          ('table1', self.waypoints[1]),
                          ('table2', self.waypoints[2]),
                          ('table3', self.waypoints[3]),
                          ('table4', self.waypoints[4]),
                          ('table5', self.waypoints[5]),
                          ('table6', self.waypoints[6]))
        
        # Store the mapping as an ordered dictionary so we can visit the rooms in sequence
        self.room_locations = OrderedDict(room_locations)         
            
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