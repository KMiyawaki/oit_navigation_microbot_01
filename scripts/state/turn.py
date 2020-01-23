# -*- coding: utf-8 -*-
import math
import rospy
import angles
import smach
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from state.wait_for_msg_state import WaitForMsgState
from utils.times import TimeChecker
from utils.funcs import get_robot_pose


class TurnOdomByAngle(WaitForMsgState):
    def __init__(self, topic, timeout=None, msg_wait=1.0, cmd_vel="/cmd_vel", min_vel=math.radians(2.5), max_vel=math.radians(30)):
        WaitForMsgState.__init__(
            self, topic, Odometry, timeout, msg_wait, _input_keys=['turn_angle'])
        self.cmd_vel = cmd_vel
        min_vel = abs(min_vel)
        max_vel = abs(max_vel)
        self.min_velocity = min(min_vel, max_vel)
        self.max_velocity = max(min_vel, max_vel)

    def __get_yaw(self, odometry):
        q = odometry.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        return yaw

    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self.__class__.__name__)
        try:
            rospy.loginfo('Max ang vel:{}, Min ang vel:{}, Turn angle={}'.format(
                math.degrees(self.min_velocity), math.degrees(self.max_velocity), math.degrees(userdata.turn_angle)))
            pub = rospy.Publisher(self.cmd_vel, Twist, queue_size=10)
            odom = Odometry()
            odom = self.get_message()
            if odom is None:
                rospy.logerr('Cannot get oeometry')
                return 'ng'
            theta_pre = self.__get_yaw(odom)
            theta_sum = 0
            theta_sum_max = abs(userdata.turn_angle)
            sign = 1
            if userdata.turn_angle < 0:
                sign = -1
            vel = Twist()
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            self.time_checker.start()
            while self.time_checker.is_timeout() is False:
                odom = self.get_message()
                if odom is None:
                    continue
                theta = self.__get_yaw(odom)
                d_theta = angles.normalize_angle(theta - theta_pre)
                theta_sum = theta_sum + abs(d_theta)
                theta_pre = theta
                finished = False
                if theta_sum >= theta_sum_max:
                    vel.angular.z = 0
                    finished = True
                else:
                    vel_abs = theta_sum_max - theta_sum
                    vel_abs = min(self.max_velocity, vel_abs)
                    vel_abs = max(self.min_velocity, vel_abs)
                    vel.angular.z = sign * vel_abs
                rospy.loginfo('Ang vel:{}, Turn sum={}, Turn to={}'.format(
                    math.degrees(vel.angular.z), math.degrees(theta_sum), math.degrees(theta_sum_max)))
                pub.publish(vel)
                if finished:
                    return 'ok'
            return 'timeout'
        except Exception as e:
            rospy.logerr(str(e))
            return 'ng'
