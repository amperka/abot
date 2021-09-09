#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String, Bool
from sound_play.libsoundplay import SoundClient

class FestvoxTTS(object):
	def __init__(self):
		rospy.init_node('festvox_tts')
		rospy.on_shutdown(self.shutdown)
		self._volume = rospy.get_param('~volume', 1.0)
		self._voice = rospy.get_param('~voice', 'voice_msu_ru_nsh_clunits')
		self._soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		rospy.Subscriber('/abot/tts/text_to_say', String, self.processText)
		self._pub = rospy.Publisher('/abot/tts/speaking_in_progress', Bool, queue_size=1)
		rospy.loginfo("Festival TTS node: Start")
		rospy.spin()

	def processText(self, text_msg):
		rospy.loginfo("Festival TTS node: Got a string: %s", text_msg.data)
		self._pub.publish(True)
		self._soundhandle.say(text_msg.data, self._voice, self._volume)
		self._pub.publish(False)

	@staticmethod
	def shutdown():
		rospy.loginfo("Festival TTS node: Stop")
		rospy.sleep(1)

if __name__ == "__main__":
	FestvoxTTS()