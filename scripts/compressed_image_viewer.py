#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import os
import cv2
import rospy
from threading import Lock
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class CompressedImageViewer(object):
    def __init__(self, image_topic_in, scale):
        rospy.Subscriber(image_topic_in, CompressedImage,
                         self.__image_callback, queue_size=1)
        self.__scale = scale
        self.__bridge = CvBridge()
        self.__winname = image_topic_in
        self.__lock_image = Lock()
        self.__img_msg = None
        cv2.namedWindow(self.__winname)

    def __get_cv_image(self):
        cv_image = None
        self.__lock_image.acquire()
        if self.__img_msg is not None:
            cv_image = self.__bridge.compressed_imgmsg_to_cv2(
                self.__img_msg, 'bgr8')
        self.__lock_image.release()
        return cv_image

    def __image_callback(self, data):
        try:
            self.__lock_image.acquire()
            self.__img_msg = copy.deepcopy(data)
            self.__lock_image.release()
        except CvBridgeError as e:
            rospy.logerr(e)

    def spin(self):
        cv_image = self.__get_cv_image()
        if cv_image is not None:
            w, h = cv_image.shape[:2]
            size = (int(h * self.__scale), int(w * self.__scale))
            cv_image = cv2.resize(cv_image, size)
            cv2.imshow(self.__winname, cv_image)
        cv2.waitKey(1)

    def stop(self):
        cv2.destroyAllWindows()


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    image_topic_in = rospy.get_param("~compressed_image_topic", "/jetbot_camera_modified_image_republisher/image/compressed")
    scale = rospy.get_param("~scale", 0.4)
    process_rate = rospy.get_param("~process_rate", 15.0)
    r = rospy.Rate(process_rate)
    node = CompressedImageViewer(image_topic_in, scale)
    while not rospy.is_shutdown():
        node.spin()
        r.sleep()
    node.stop()


if __name__ == '__main__':
    main()
