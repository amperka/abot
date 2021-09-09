#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String, Empty
from audio_common_msgs.msg import AudioData
from pocketsphinx import Decoder, Jsgf

class ASRControl(object):
	def __init__(self):
		rospy.init_node("asr_control")
		rospy.on_shutdown(self.shutdown)
		self._grammar_data_pub = rospy.Publisher("/abot/stt/grammar_data", String, queue_size=10)
		self._grammar_not_found_pub = rospy.Publisher('/abot/stt/grammar_not_found', Empty, queue_size=10)
		self._hmm = rospy.get_param('~hmm')
		self._dict = rospy.get_param('~dict')
		self._gram = rospy.get_param('~gram')
		self._rule = rospy.get_param('~rule')
		self._in_speech_bf = False
		self.startRecognizer()

	def startRecognizer(self):
		config = Decoder.default_config()
		config.set_string('-hmm', self._hmm)
		config.set_string('-dict', self._dict)
		self._decoder = Decoder(config)
		jsgf = Jsgf(self._gram + '.gram')
		rule = jsgf.get_rule(rospy.get_param('~grammar') + '.' + self._rule)
		fsg = jsgf.build_fsg(rule, self._decoder.get_logmath(), 7.5)
		fsg.writefile(self._gram + '.fsg')
		self._decoder.set_fsg(self._gram, fsg)
		self._decoder.set_search(self._gram)
		self._decoder.start_utt()
		rospy.loginfo("ASR control node: Decoder started successfully")
		rospy.Subscriber("/abot/stt/grammar_audio", AudioData, self.processAudio)
		rospy.spin()

	def processAudio(self, audio_msg):
		self._decoder.process_raw(audio_msg.data, False, False)
		if self._decoder.get_in_speech() != self._in_speech_bf:
			self._in_speech_bf = self._decoder.get_in_speech()
			if not self._in_speech_bf:
				self._decoder.end_utt()
				if self._decoder.hyp() is not None:
					msg = self._decoder.hyp().hypstr
					rospy.logwarn('ASR control node: OUTPUT - \"' + msg + '\"')
					self._grammar_data_pub.publish(msg)
				else:
					rospy.logwarn("ASR control node: No possible grammar found")
					msg = Empty()
					self._grammar_not_found_pub.publish(msg)
				self._decoder.start_utt()

	@staticmethod
	def shutdown():
		rospy.loginfo("ASR control node: Stop ASRControl")
		rospy.sleep(1)

if __name__ == "__main__":
	ASRControl()