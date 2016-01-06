#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class SoundIndicator():
    def __init__(self):
        rospy.init_node('my_soundmsgs_listener', anonymous=True)
        rospy.Subscriber('sound_message', String, self.my_callback_function)

	soundhandle = SoundClient()

        self.sound1 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/sound/smb_coin.wav')
        self.sound2 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/sound/smb_1-up.wav')
        self.sound3 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/sound/smb_stage_clear.wav')
        self.sound4 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/sound/smb_mariodie.wav')
        self.sound5 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/sound/smb_powerup.wav')


	self.welcome = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/welcome.wav')
	self.waiting_order = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/waiting_order.wav')
	self.O1 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/O1.wav')
	self.O2 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/O2.wav')
	self.O3 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/O3.wav')
	self.O4 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/O4.wav')
	self.O5 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/O5.wav')
	self.O6 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/O6.wav')
	self.M1 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/M1.wav')
	self.M2 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/M2.wav')
	self.M3 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/M3.wav')
	self.M4 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/M4.wav')
	self.M5 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/M5.wav')
	self.M6 = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/M6.wav')
	self.table_search = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/table_search.wav')
	self.thanks = soundhandle.waveSound('/home/turtlebot/ros/hydro/catkin_ws/src/rbx2/rbx2_tasks/new_sound_wav/thanks.wav')

    
    def my_callback_function(self, data):

        if (data.data == "sound_option1"):
            self.sound1.play()
        elif (data.data == "sound_option2"):
            self.sound2.play()
        elif (data.data == "sound_option3"):
            self.sound3.play()
        elif (data.data == "sound_option4"):
            self.sound4.play()
        elif (data.data == "sound_option5"):
            self.sound5.play()

	elif (data.data == "welcome"):
	    self.welcome.play()
	elif (data.data == "waiting_order"):
	    self.waiting_order.play()
	elif (data.data == "O1"):
	    self.O1.play()
	elif (data.data == "O2"):
	    self.O2.play()
	elif (data.data == "O3"):
	    self.O3.play()
	elif (data.data == "O4"):
	    self.O4.play()
	elif (data.data == "O5"):
	    self.O5.play()
	elif (data.data == "O6"):
	    self.O6.play()
	elif (data.data == "M1"):
	    self.M1.play()
	elif (data.data == "M2"):
	    self.M2.play()
	elif (data.data == "M3"):
	    self.M3.play()
	elif (data.data == "M4"):
	    self.M4.play()
	elif (data.data == "M5"):
	    self.M5.play()
	elif (data.data == "M6"):
	    self.M6.play()
	elif (data.data == "table_search"):
	    self.table_search.play()
	elif (data.data == "thanks"):
	    self.thanks.play()

if __name__ == '__main__':
    try:
        SoundIndicator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('SoundIndicator Finish')
        


