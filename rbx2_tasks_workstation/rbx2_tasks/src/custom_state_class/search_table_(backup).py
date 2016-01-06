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

class SearchTable(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect','not_detect'])
        self.taskname="doing the computer vision task"

        # initialize a subscriber of ar_pose_marker
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.my_listener)
        self.tag_ids = 6
        self.n_markers = 0
        self.my_tagid = 0

        self.table_flag = False;

        self.my_tagposition_y = 999

    def execute(self, userdata):
    	rospy.loginfo("execute SearchTable class")

        # while( self.n_markers==0 or not(self.my_tagposition_y > -0.2 and self.my_tagposition_y < 0.2)):
        while( not(self.table_flag) ):

            if self.preempt_requested():
                rospy.loginfo("State SearchTable is being preempted!!!")
                self.service_preempt()
                return 'not_detect'

            rospy.loginfo("Searching for table...")
            rospy.sleep(1)

        rospy.loginfo("Table detected!")
        self.table_flag = False;
        return 'detect'

    def my_listener(self, msg):

        # get the number of markers
        self.n_markers = len(msg.markers)

        # if there is no marker stop the listener
        # at least we already have the number of markers
        if self.n_markers == 0:
            return

        # assuming there are only 1 tag at once 
        for tag in msg.markers:

            if tag.id is 6:
	            #self.my_tagid = tag.id
	            rospy.loginfo("detectec: " +str(tag.id))
	            self.table_flag = True
	            self.my_tagposition_y = tag.pose.pose.position.y

'''
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

class SearchTable(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect','not_detect'])
        self.taskname="doing the computer vision task"

        # initialize a subscriber of ar_pose_marker
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.my_listener)
        self.tabletype_dictionary = {6:'Table VIP', 5:'Normal'}

        self.n_markers = 0
        self.my_tagid = 0

        self.my_tagposition_y = 999

    def execute(self, userdata):

        # while( self.n_markers==0 or not(self.my_tagposition_y > -0.2 and self.my_tagposition_y < 0.2)):
        while(self.n_markers==0 or self.my_tagid == 6):
            if self.preempt_requested():
                rospy.loginfo("State SearchTable is being preempted!!!")
                self.service_preempt()
                return 'not_detect'

            rospy.loginfo("Searching for table... | "+ str(self.my_tagid))
            rospy.sleep(1)

        rospy.loginfo("Table detected!")
        return 'detect'

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
            if self.tabletype_dictionary is not None and not tag.id in self.tabletype_dictionary:
                continue

            rospy.loginfo("Marker detected : "+str(tag.id))
            self.my_tagid = tag.id
            self.my_tagposition_y = tag.pose.pose.position.y
'''