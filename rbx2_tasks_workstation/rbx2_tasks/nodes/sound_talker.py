#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('sound_message', String, queue_size=10)
    rospy.init_node('my_soundmsgs_talker', anonymous=True)

    while not rospy.is_shutdown():
        hello_str = "welcome"
        pub.publish(hello_str)
        rospy.sleep(8)
        hello_str = "waiting_order"
        pub.publish(hello_str)
        rospy.sleep(3)
        hello_str = "table_search"
        pub.publish(hello_str)
        rospy.sleep(4)
        hello_str = "thanks"
        pub.publish(hello_str)
        rospy.sleep(5)
        hello_str = "O1"
        pub.publish(hello_str)
        rospy.sleep(4)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass