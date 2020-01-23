# -*- coding: utf-8 -*-
import cv2
import rospy
import smach
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class GetImageState(smach.State):
    def __init__(self, topic, file_path, msg_wait=50.0, _input_keys=[], _output_keys=[]):
        smach.State.__init__(
            self, outcomes=['ok', 'ng'], input_keys=_input_keys, output_keys=_output_keys)
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = Image
        self.file_path = file_path

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
        message = self.get_message()
        try:
            if message is not None:
                cv_image = CvBridge().imgmsg_to_cv2(message, "bgr8")
                cv2.imwrite(self.file_path, cv_image)
                rospy.loginfo(self.__class__.__name__ + ':save image file into .' + self.file_path)
                return 'ok'
            else:
                rospy.logerr(self.__class__.__name__ + ':Failed to get image.')
        except:
            rospy.logerr(self.__class__.__name__ + ':Failed to get image.')
        return 'ng'
