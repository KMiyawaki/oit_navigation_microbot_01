#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class ImageModifier(object):
    def __init__(self, frame_id, image_topic_in, image_topic_raw):
        self.__frame_id = frame_id
        rospy.Subscriber(image_topic_in, Image,
                         self.__image_callback, queue_size=1)
        self.__pub_image_raw = rospy.Publisher(
            image_topic_raw, Image, queue_size=1)
        self.__bridge = CvBridge()

    def __image_callback(self, data):
        try:
            cv_image = cv2.flip(self.__bridge.imgmsg_to_cv2(data, "bgr8"), -1)
            data_mod = self.__bridge.cv2_to_imgmsg(cv_image, "bgr8")
            data_mod.header.stamp = rospy.Time.now()
            data_mod.header.frame_id = self.__frame_id
            self.__pub_image_raw.publish(data_mod)
        except CvBridgeError as e:
            rospy.logerr(e)


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    frame_id = "camera_link"
    image_topic_in = "/jetbot_camera/raw"
    image_topic_raw = "~mod_image"
    _ = ImageModifier(frame_id, image_topic_in,
                      image_topic_raw)
    rospy.spin()


if __name__ == '__main__':
    main()
