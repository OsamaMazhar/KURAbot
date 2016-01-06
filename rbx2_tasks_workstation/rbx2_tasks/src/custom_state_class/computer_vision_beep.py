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
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

class ComputerVision(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect1','detect2','detect3','detect4','detect5','detect6', 'preempted'])
        self.taskname="doing the computer vision task"

        # initialize a subscriber of ar_pose_marker
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.my_listener)
        self.tag_ids = [1,2,3,4,5,6]
        self.execute_flag = False

        self.marker_table_dictionary = {1:'detect1', 2:'detect2', 3:'detect3', 4:'detect4', 5:'detect5', 6:'detect6'}
        self.my_outcome = self.marker_table_dictionary[1]
        self.n_markers = 0
        self.my_tagid = 0

        self.soundstring_publisher = rospy.Publisher('sound_message', String, queue_size=10)

    def execute(self, userdata):
        self.execute_flag = True
        self.soundstring_publisher.publish("waiting_order")
        rospy.sleep(3);

        # while there is no marker, hold the execution of execute function
        while(self.n_markers==0):

            if self.preempt_requested():
                rospy.loginfo("State ComputerVision is being preempted!!!")
                self.service_preempt()
                return 'preempted'

            rospy.loginfo("waiting for delivery command...")
            rospy.sleep(1)

        # right after there is marker return an outcome
        rospy.loginfo("number detected! it is: " + str(self.my_tagid) )

        if (self.my_tagid==1):
            self.soundstring_publisher.publish("O1")
            rospy.sleep(3)
            self.soundstring_publisher.publish("M1")
            rospy.sleep(3)
        elif (self.my_tagid==2):
            self.soundstring_publisher.publish("O2")
            rospy.sleep(3)
            self.soundstring_publisher.publish("M2")
            rospy.sleep(3)
        elif (self.my_tagid==3):
            self.soundstring_publisher.publish("O3")
            rospy.sleep(3)
            self.soundstring_publisher.publish("M3")
            rospy.sleep(3)
        elif (self.my_tagid==4):
            self.soundstring_publisher.publish("O4")
            rospy.sleep(3)
            self.soundstring_publisher.publish("M4")
            rospy.sleep(3)
        elif (self.my_tagid==5):
            self.soundstring_publisher.publish("O5")
            rospy.sleep(3)
            self.soundstring_publisher.publish("M5")
            rospy.sleep(3)
        else:
            self.soundstring_publisher.publish("O6")
            rospy.sleep(3)
            self.soundstring_publisher.publish("M6")
            rospy.sleep(3)


        self.reset_class_attribute()
        return self.my_outcome

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

                # Skip any tags that are not in our list
                if self.tag_ids is not None and not tag.id in self.tag_ids:
                    continue

                self.my_tagid   = tag.id
                self.my_outcome = self.marker_table_dictionary[tag.id]

    # this function will reset every attribute of this class 
    def reset_class_attribute(self):
        self.execute_flag = False
        self.n_markers = 0

'''
class ComputerVision(State):
    def __init__(self):
        State.__init__(self, outcomes=['detect1','detect2','detect3','detect4','detect5','detect6','preempted'])
        self.taskname="doing the computer vision task"

        #soundhandle = SoundClient()
        #self.detect_tableorder = soundhandle.waveSound('/home/mscv/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/sound/smb_coin.wav')

        self.soundstring_publisher = rospy.Publisher('sound_message', String, queue_size=10)

    def execute(self, userdata):
        self.soundstring_publisher.publish("waiting_order")

        rospy.loginfo("detecing number...")
        rospy.sleep(3)
        table = random.randint(1,2)
        rospy.loginfo("number detected! it is: " + str(table) )

        if (table==1):
            self.soundstring_publisher.publish("O1")
            rospy.sleep(4)
            self.soundstring_publisher.publish("M1")
            rospy.sleep(4)
            return 'detect1'
        elif (table==2):
            self.soundstring_publisher.publish("O2")
            rospy.sleep(4)
            self.soundstring_publisher.publish("M2")
            rospy.sleep(4)
            return 'detect2'
        elif (table==3):
            self.soundstring_publisher.publish("O3")
            rospy.sleep(4)
            self.soundstring_publisher.publish("M3")
            rospy.sleep(4)
            return 'detect4'
        elif (table==4):
            self.soundstring_publisher.publish("O4")
            rospy.sleep(4)
            self.soundstring_publisher.publish("M4")
            rospy.sleep(4)
            return 'detect4'
        elif (table==5):
            self.soundstring_publisher.publish("O5")
            rospy.sleep(4)
            self.soundstring_publisher.publish("M5")
            rospy.sleep(4)
            return 'detect5'
        else:
            self.soundstring_publisher.publish("O6")
            rospy.sleep(4)
            self.soundstring_publisher.publish("M6")
            rospy.sleep(4)
            return 'detect6'
'''

