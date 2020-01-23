#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import smach
import tf
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from state.wait_for_msg_state import WaitForMsgState
from state.utils.funcs import get_robot_pose
from state.utils.trapezoidal_control import TrapezoidalControl
from state.utils.times import TimeChecker


class GoStraightOdomByDistance(WaitForMsgState):
    def __init__(self, topic, timeout=None, msg_wait=1.0, cmd_vel="/cmd_vel", acceleration=0.1, min_vel_norm=0.01):
        WaitForMsgState.__init__(
            self, topic, Odometry, timeout, msg_wait, _input_keys=['linear_x', 'linear_y', 'distance'])
        self.cmd_vel = cmd_vel
        self.acceleration = abs(acceleration)
        self.min_vel_norm = abs(min_vel_norm)

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        self.velocity_vector_crnt = np.array([0, 0])
        try:
            rospy.loginfo('Velocities({}, {}), Distance={}'.format(
                userdata.linear_x, userdata.linear_y, userdata.distance))
            pub = rospy.Publisher(self.cmd_vel, Twist, queue_size=10)
            odom = Odometry()
            odom = self.get_message()
            if odom is None:
                rospy.logerr('Cannot get oeometry')
                return 'ng'

            trapezoidal_control = TrapezoidalControl(
                userdata.linear_x, userdata.linear_y, self.acceleration, self.min_vel_norm)
            trapezoidal_control.calc_controle_profiles(userdata.distance)
            distance_profile = trapezoidal_control.distance_profiles

            first_x = odom.pose.pose.position.x
            first_y = odom.pose.pose.position.y
            vel = Twist()
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            self.time_checker.start()
            while self.time_checker.is_timeout() is False:
                delta = self.time_checker.delta_time()
                # elapsed = self.time_checker.elapsed()
                odom = self.get_message()
                if odom is None:
                    continue
                x = odom.pose.pose.position.x
                y = odom.pose.pose.position.y
                dx = x - first_x
                dy = y - first_y
                d = math.sqrt((dx * dx) + (dy * dy))
                finished = False
                if d > userdata.distance:
                    vel.linear.x = 0
                    vel.linear.y = 0
                    finished = True
                else:
                    acc = self.acceleration
                    if d >= distance_profile[0] + distance_profile[1]:
                        acc = -self.acceleration
                    self.velocity_vector_crnt = trapezoidal_control.calc_velocity_vector(self.velocity_vector_crnt,
                                                                                         acc, delta)
                    vel.linear.x = self.velocity_vector_crnt[0]
                    vel.linear.y = self.velocity_vector_crnt[1]
                rospy.logdebug(
                    "{}, {} -> {}, {}, velocities=({}, {}) moved distance={}".format(first_x, first_y, x, y, vel.linear.x, vel.linear.y, d))
                pub.publish(vel)
                if finished:
                    return 'ok'
            return 'timeout'
        except Exception as e:
            rospy.logerr(str(e))
            return 'ng'
