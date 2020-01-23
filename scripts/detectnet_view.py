#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import os
import cv2
import rospy
from threading import Lock
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge, CvBridgeError


class DetectnetView(object):
    def __init__(self, image_topic_in="/jetbot_camera_image_modifier/mod_image",
                 detection_topic_in="/detectnet/detections",
                 image_topic_out="~detected_image",
                 labels=None,
                 line_width=3,
                 font_scale=4):
        rospy.Subscriber(image_topic_in, Image,
                         self.__image_callback, queue_size=1)
        rospy.Subscriber(detection_topic_in, Detection2DArray,
                         self.__detection_callback, queue_size=1)
        self.__pub_image = rospy.Publisher(
            image_topic_out, Image, queue_size=1)
        self.__bridge = CvBridge()
        self.__lock_image = Lock()
        self.__image = None
        self.__lock_detection = Lock()
        self.__detection = None
        self.__line_width = line_width
        self.__labels = labels
        self.__color = (0, 0, 255)
        self.__font_scale = font_scale

    def __image_callback(self, data):
        try:
            self.__lock_image.acquire()
            self.__image = data
            self.__lock_image.release()
        except Exception as e:
            rospy.logerr(e)

    def __detection_callback(self, data):
        try:
            self.__lock_detection.acquire()
            if data is not None:
                self.__detection = data
                self.__detection.header.stamp = rospy.Time.now()
            self.__lock_detection.release()
        except Exception as e:
            rospy.logerr(e)

    def __get_cv_image(self):
        header = None
        cv_image = None
        try:
            self.__lock_image.acquire()
            if self.__image is not None:
                cv_image = self.__bridge.imgmsg_to_cv2(self.__image, "bgr8")
                header = copy.deepcopy(self.__image.header)
            self.__lock_image.release()
        except Exception as e:
            rospy.logerr(e)
        return (header, cv_image)

    def __get_detection_result(self):
        detection_result = None
        try:
            self.__lock_detection.acquire()
            if self.__detection is not None:
                d = rospy.Time.now() - self.__detection.header.stamp
                if d.to_sec() > 2:
                    self.__detection = None
                else:
                    detection_result = copy.deepcopy(self.__detection)
            self.__lock_detection.release()
        except Exception as e:
            rospy.logerr(e)
        return detection_result

    def __draw_object(self, cv_image, detection_2d):
        cx = detection_2d.bbox.center.x
        cy = detection_2d.bbox.center.y
        x_2 = detection_2d.bbox.size_x / 2.0
        y_2 = detection_2d.bbox.size_y / 2.0
        lt = (int(cx - x_2), int(cy - y_2))
        rb = (int(cx + x_2), int(cy + y_2))
        text_origin = (int(cx - x_2), int(cy - y_2 - self.__line_width))
        class_id = detection_2d.results[0].id
        try:
            label = self.__labels[class_id]
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, str(class_id) + ":" + label, text_origin, font,
                        self.__font_scale, self.__color, self.__line_width, cv2.LINE_AA)
            cv2.rectangle(cv_image, lt, rb, self.__color, self.__line_width)
        except Exception as e:
            rospy.logerr(e)

    def __draw_objects(self, cv_image, detection_2d_array):
        for d in detection_2d_array.detections:
            self.__draw_object(cv_image, d)

    def spin(self):
        (header, cv_image) = self.__get_cv_image()
        detection = self.__get_detection_result()
        if header is None or cv_image is None:
            return
        if detection is not None:
            self.__draw_objects(cv_image, detection)
        result_image = self.__bridge.cv2_to_imgmsg(cv_image, "bgr8")
        result_image.header = header
        self.__pub_image.publish(result_image)


def load_labels(labels_txt):
    labels = None
    try:
        with open(labels_txt) as f:
            labels = [s.strip() for s in f.readlines()]
    except Exception as e:
        rospy.logerr(e)
    return labels


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    image_topic_in = "/jetbot_camera_image_modifier/mod_image"
    detection_topic_in = "/detectnet/detections"
    image_topic_out = "~detected_image"
    process_rate = rospy.get_param("~process_rate", 20.0)
    label_file = rospy.get_param(
        "~object_labels", "/usr/local/bin/networks/SSD-Mobilenet-v2/ssd_coco_labels.txt")
    labels = load_labels(label_file)
    if labels is None:
        return
    line_width = rospy.get_param("~line_width", 3)
    font_scale = rospy.get_param("~font_scale", 3)
    node = DetectnetView(image_topic_in,
                         detection_topic_in,
                         image_topic_out,
                         labels,
                         line_width,
                         font_scale)
    rate = rospy.Rate(process_rate)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
