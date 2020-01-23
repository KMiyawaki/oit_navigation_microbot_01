#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy


class TrapezoidalControl(object):
    def __init__(self, linear_x, linear_y, acceleration, min_vel_norm):
        self.velocity_vector_max = np.array([linear_x, linear_y])
        self.acceleration = abs(acceleration)
        self.min_vel_norm = abs(min_vel_norm)
        self.max_vel_norm = np.linalg.norm(self.velocity_vector_max)
        self.time_profiles = None
        self.distance_profiles = None

    def calc_velocity_norm(self, velocity_norm_crnt, acceleration, delta_time, min_vel_norm, max_vel_norm):
        norm = velocity_norm_crnt + acceleration * delta_time
        norm = max(min_vel_norm, norm)
        norm = min(max_vel_norm, norm)
        return norm

    def calc_velocity_vector(self, velocity_vector_crnt, acceleration, delta_time):
        velocity_norm_crnt = np.linalg.norm(velocity_vector_crnt)
        velocity_norm = self.calc_velocity_norm(
            velocity_norm_crnt, acceleration, delta_time, self.min_vel_norm, self.max_vel_norm)
        return self.velocity_vector_max * (velocity_norm / self.max_vel_norm)

    def calc_controle_profiles(self, distance):
        # https://tajimarobotics.com/acceleration-limited-feed-profile-no-cruise/
        time_span_1 = self.max_vel_norm / self.acceleration
        length_1 = time_span_1 * self.max_vel_norm / 2.0
        time_span_3 = time_span_1
        length_3 = length_1
        length_2 = distance - length_1 - length_3
        time_span_2 = length_2 / self.max_vel_norm
        vel_p = self.max_vel_norm
        if length_2 < 0:
            length_2 = 0
            vel_p = math.sqrt(self.acceleration * distance)
            time_span_1 = vel_p / self.acceleration
            time_span_3 = time_span_1
            time_span_2 = 0
        rospy.loginfo("Trapezoidal control profile. Time span 1:{}, Time span 2:{}, Time span 3:{}, Fp:{}".format(
            time_span_1, time_span_2, time_span_3, vel_p))
        rospy.loginfo("Trapezoidal control profile. Distance 1:{}, Distance 2:{}, Distance 3:{}".format(
            length_1, length_2, length_3))
        self.time_profiles = (time_span_1, time_span_2, time_span_3)
        self.distance_profiles = (length_1, length_2, length_3)
        return (self.time_profiles, self.distance_profiles)
