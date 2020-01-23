# -*- coding: utf-8 -*-
import rospy
import smach
from utils.times import TimeChecker


class WaitForMsgState(smach.State):
    def __init__(self, topic, msg_type, timeout=None, msg_wait=50.0, _input_keys=[], _output_keys=[]):
        smach.State.__init__(
            self, outcomes=['ok', 'ng', 'timeout'], input_keys=_input_keys, output_keys=_output_keys)
        self.time_checker = TimeChecker(timeout)
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type
        self.finished = False
        self.result = 'timeout'

    def on_message(self, msg, userdata):
        rospy.loginfo(self.get_message_info(msg))

    def get_message_info(self, msg):
        text = 'Recv sensor. frame_id = {}, seq = {}'.format(
            msg.header.frame_id, msg.header.seq)
        return text

    def get_message(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logerr(e)
        return message

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        self.time_checker.start()
        while self.time_checker.is_timeout() is False and self.finished is False:
            message = self.get_message()
            if message is None:
                continue
            if self.on_message(message, userdata) is True:
                self.result = 'ok'
                return self.result
        return self.result
