#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String, Empty, Bool
from audio_common_msgs.msg import AudioData
from pocketsphinx import Decoder

class KWSDetection(object):
	def __init__(self):
		rospy.init_node('kws_control')
		rospy.on_shutdown(self.shutdown)
		self._kws_data_pub = rospy.Publisher('/abot/stt/kws_data', String, queue_size=10)
		self._grammar_audio_pub = rospy.Publisher('/abot/stt/grammar_audio', AudioData, queue_size=10)
		self._hmm = rospy.get_param('~hmm')
		self._dict = rospy.get_param('~dict')
		self._kws = rospy.get_param('~kws')
		self._buffer_index = 0
		self._buffer = bytearray()
		self._kws_found = False
		self._tts_is_speaking = False
		self.startRecognizer()

	def startRecognizer(self):
		config = Decoder.default_config()
		config.set_string('-hmm', self._hmm)
		config.set_string('-dict', self._dict)
		config.set_string('-kws', self._kws)
		self._decoder = Decoder(config)
		self._decoder.start_utt()
		rospy.loginfo("KWS control node: Decoder started successfully")
		rospy.Subscriber('/audio', AudioData, self.makeBuffer)
		rospy.Subscriber('/abot/stt/grammar_not_found', Empty, self.grammarCheckCallback)
		rospy.Subscriber("/abot/tts/speaking_in_progress", Bool, self.ttsSpeakingCheck)
		rospy.spin()

	def grammarCheckCallback(self, empty_msg):
		self._kws_found = False
		rospy.logwarn("KWS control node: Stop ASR audio transmission")

	def ttsSpeakingCheck(self, bool_msg):
		self._tts_is_speaking = bool_msg.data
		if self._tts_is_speaking:
			rospy.loginfo("KWS control node: Decoder paused while TTS is speaking")
		else:
			rospy.loginfo("KWS control node: Decoder returned to work")

	def makeBuffer(self, audio_msg):
		self._buffer += audio_msg.data
		self._buffer_index = self._buffer_index + 1
		if self._buffer_index == 3:
			self.processAudio(self._buffer)
			self._buffer = bytearray()
			self._buffer_index = 0

	def processAudio(self, audio_buffer):
		self._decoder.process_raw(audio_buffer, False, False)

		if self._tts_is_speaking is False:
			if self._decoder.hyp() is not None:
				for seg in self._decoder.seg():
					rospy.logwarn("Detected key words: %s ", seg.word)
					self._decoder.end_utt()
					msg = seg.word
					self._kws_data_pub.publish(msg)
					self._kws_found = True
					self._decoder.start_utt()

			if self._kws_found is True:
				msg = AudioData()
				msg.data = audio_buffer
				self._grammar_audio_pub.publish(msg)

	@staticmethod
	def shutdown():
		rospy.loginfo("KWS control node: Stop KWSDetection")
		rospy.sleep(1)

if __name__ == "__main__":
	KWSDetection()