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

import random

class ComputerVision(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect1','detect2','detect3','detect4','detect5','detect6'])
        self.taskname="doing the computer vision task"

    def execute(self, userdata):
        rospy.loginfo("detecing number...")
        rospy.sleep(5)
        table = random.randint(1,1)
        rospy.loginfo("number detected! it is: " + str(table) )

        if (table==1):
            return 'detect1'
        elif (table==2):
            return 'detect2'
        elif (table==3):
            return 'detect4'
        elif (table==4):
            return 'detect4'
        elif (table==5):
            return 'detect5'
        else:
            return 'detect6'

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
'''
