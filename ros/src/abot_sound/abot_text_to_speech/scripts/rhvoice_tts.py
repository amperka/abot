#!/usr/bin/env python
# coding: utf-8

import rospy
import subprocess
from std_msgs.msg import String, Bool
from sound_play.libsoundplay import SoundClient

class RHVoiceTTS(object):
	def __init__(self):
		rospy.init_node('rhvoice_tts')
		rospy.on_shutdown(self.shutdown)
		self._volume = rospy.get_param('~volume', 1.0)
		self._rhvoice_speech_sound_file_path = rospy.get_param('~rhvoice_speech_sound_file_path')
		self._rhvoice_voice = rospy.get_param('~rhvoice_voice', 'Anna+CLB')
		self._soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		rospy.Subscriber('/abot/tts/text_to_say', String, self.processText)
		self._pub = rospy.Publisher('/abot/tts/speaking_in_progress', Bool, queue_size=1)
		rospy.loginfo("RHVoice TTS node: Start")
		rospy.spin()

	def processText(self, text_msg):
		rospy.loginfo("RHVoice TTS node: Got a string: %s", text_msg.data)
		rospy.loginfo("RHVoice TTS node: Saving speech to file: %s", self._rhvoice_speech_sound_file_path)
		rhvoice_command_line = "echo '" + text_msg.data + "' | RHVoice-client -s " + self._rhvoice_voice + " "
		rhvoice_command_line += "| sox -t wav - -r 24000 -c 1 -b 16 -t wav - >" + self._rhvoice_speech_sound_file_path
		rospy.loginfo("RHVoice TTS node: Command: %s", rhvoice_command_line)
		subprocess.call(rhvoice_command_line, shell=False)
		self._pub.publish(True)
		rospy.loginfo('RHVoice TTS node: Playing "%s".', self._rhvoice_speech_sound_file_path)
		self._soundhandle.playWave(self._rhvoice_speech_sound_file_path, self._volume)
		self._pub.publish(False)
		rospy.loginfo('RHVoice TTS node: Stop Playing')

	@staticmethod
	def shutdown():
		rospy.loginfo("RHVoice TTS node: Stop")
		rospy.sleep(1)

if __name__ == "__main__":
	RHVoiceTTS()
