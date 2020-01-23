#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import tf
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseGoal
from tf.transformations import euler_from_quaternion


class RobotName(object):
    def ORION(self):
        return 'orion'

    def RASALAS(self):
        return 'rasalas'


def is_real_robot(param_name='~use_actuators'):
    return rospy.get_param(param_name)


def get_robot_name(param_name='~robot_name'):
    return rospy.get_param(param_name)


def radians_array(degrees):
    radians = []
    for d in degrees:
        radians.append(math.radians(d))
    return radians


def get_robot_pose(tf_listener, target_frame='map', source_frame='base_link', timeout=5.0):
    tf_listener.waitForTransform(
        target_frame, source_frame, rospy.Time(), rospy.Duration(timeout))
    (trans, rot) = tf_listener.lookupTransform(
        target_frame, source_frame, rospy.Time(0))
    x = trans[0]
    y = trans[1]
    (_, _, yaw) = euler_from_quaternion(rot)
    return (x, y, yaw)


def get_angle(target, source):
    x = target[0] - source[0]
    y = target[1] - source[1]

    rospy.loginfo('x = ' + str(x))
    rospy.loginfo('y = ' + str(y))

    rad = math.atan2(y, x)

    rospy.loginfo('rad = ' + str(math.degrees(rad)))

    return rad


def make_move_base_goal(target_frame, stamp, x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = target_frame
    goal.target_pose.header.stamp = stamp
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    return goal
