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

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

class SearchTable(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect','not_detect'])

        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.my_listener)
        self.n_markers = 0
        self.table_flag = False
        self.execute_flag = False

        self.sum_x = 0
        self.counter = 0
        self.mean_distance = 0

        self.soundstring_publisher = rospy.Publisher('sound_message', String, queue_size=10)

    def execute(self, userdata):
        self.execute_flag = True
        rospy.loginfo("execute SearchTable class")

        self.soundstring_publisher.publish("table_search")
        rospy.sleep(4)

        # while( self.n_markers==0 or not(self.my_tagposition_y > -0.2 and self.my_tagposition_y < 0.2)):
        while( not(self.table_flag) or (self.mean_distance > 1) ):

            if self.preempt_requested():
                rospy.loginfo("State SearchTable is being preempted!!!")
                self.service_preempt()
                self.reset_class_attribute()

                message_str = "sound_option4" # mario die sound
                self.soundstring_publisher.publish(message_str)
                rospy.sleep(3)
                return 'not_detect'

            rospy.loginfo("Searching for table...")
            rospy.sleep(1)

        rospy.loginfo("Table detected!")
        self.reset_class_attribute()
        return 'detect'

    def my_listener(self, msg):

        if self.execute_flag:
            # get the number of markers
            self.n_markers = len(msg.markers)

            # if there is no marker stop the listener
            # at least we already have the number of markers
            if self.n_markers == 0:
                return

            # assuming there are only 1 tag at once 
            for tag in msg.markers:

                if tag.id is 7:
                    self.table_flag = True

                    self.counter=self.counter+1
                    self.sum_x = self.sum_x + tag.pose.pose.position.x
                    self.mean_distance = self.sum_x/self.counter

                    rospy.loginfo("detected: " +str(tag.id) + ", it is: "+str(tag.pose.pose.position.x) + "m")
                    rospy.loginfo("the mean is: "+str(self.mean_distance) + "m")

    # this function will reset every attribute of this class 
    def reset_class_attribute(self):
        self.table_flag = False
        self.execute_flag = False
        self.n_markers = 0

        self.sum_x = 0
        self.counter = 0
        self.mean_distance = 0

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