#!/usr/bin/env python
# coding: utf-8

import rospy
from boto3 import Session
from std_msgs.msg import String, Bool
from sound_play.libsoundplay import SoundClient

class AWSPollyTTS(object):
	def __init__(self):
		rospy.init_node('aws_polly_tts')
		rospy.on_shutdown(self.shutdown)
		self._volume = rospy.get_param('~volume', 1.0)
		self._aws_speech_sound_file_path = rospy.get_param('~aws_speech_sound_file_path')
		self._aws_access_key_id = rospy.get_param('~aws_access_key_id')
		self._aws_secret_access_key = rospy.get_param('~aws_secret_access_key')
		self._aws_region_name =rospy.get_param('~aws_region_name', 'us-west-2')
		self._aws_polly_voice_id =rospy.get_param('~aws_polly_voice_id')
		self._session = Session(aws_access_key_id=self._aws_access_key_id, aws_secret_access_key=self._aws_secret_access_key, region_name=self._aws_region_name)
		self._polly = self._session.client('polly')
		self._soundhandle = SoundClient(blocking=True)
		rospy.sleep(1)
		rospy.Subscriber('/abot/tts/text_to_say', String, self.processText)
		self._pub = rospy.Publisher('/abot/tts/speaking_in_progress', Bool, queue_size=1)
		rospy.loginfo("AWS Polly TTS node: Start")
		rospy.spin()

	def processText(self, text_msg):
		rospy.loginfo("AWS Polly TTS node: Got a string: %s", text_msg.data)
		response = self._polly.synthesize_speech(VoiceId=self._aws_polly_voice_id, OutputFormat='ogg_vorbis', Text = text_msg.data)

		rospy.loginfo("AWS Polly TTS node: Saving speech to file: %s", self._aws_speech_sound_file_path)
		file = open(self._aws_speech_sound_file_path, 'wb')
		file.write(response['AudioStream'].read())
		file.close()

		self._pub.publish(True)
		rospy.loginfo('AWS Polly TTS node: Playing "%s".', self._aws_speech_sound_file_path)
		self._soundhandle.playWave(self._aws_speech_sound_file_path, self._volume)
		self._pub.publish(False)

	@staticmethod
	def shutdown():
		rospy.loginfo("AWS Polly TTS node: Stop")
		rospy.sleep(1)

if __name__ == "__main__":
	AWSPollyTTS()
