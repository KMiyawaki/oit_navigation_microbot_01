#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import math
import os
import rospy
import tf
from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TestMotors(object):
    def __init__(self, cmd_vel_topic, odometry_topic):
        rospy.Subscriber(odometry_topic, Odometry,
                         self.__odom_callback, queue_size=1)
        self.__pub_cmd_vel = rospy.Publisher(
            cmd_vel_topic, Twist, queue_size=10)
        self.__odom = None
        self.__lock_odom = Lock()
        self.__tm = rospy.Time.now()

    def __odom_callback(self, data):
        try:
            self.__lock_odom.acquire()
            self.__odom = data
            self.__lock_odom.release()
        except Exception as e:
            rospy.logerr(e)

    def __get_odom(self):
        odom = None
        try:
            self.__lock_odom.acquire()
            if self.__odom is not None:
                odom = copy.deepcopy(self.__odom)
            self.__lock_odom.release()
        except Exception as e:
            rospy.logerr(e)
        return odom

    def __get_yaw(self, odometry):
        q = odometry.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        return yaw

    def spin(self):
        odom = self.__get_odom()
        # Todo check timestamp of 'odom' and dispose if it is too old.
        if odom is not None:
            # Display odometry
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            yaw = self.__get_yaw(odom)
            rospy.loginfo("%d.%d:(%f, %f) yaw = %f" %
                          (odom.header.stamp.secs, odom.header.stamp.nsecs, x, y, math.degrees(yaw)))
        # Publish velocities
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        lin_vel_abs = 0.1
        ang_vel_abs = math.radians(45)
        tm = rospy.Time.now()
        dt = (tm - self.__tm).to_sec()
        if dt < 2:
            vel.linear.x = lin_vel_abs
        elif dt < 4:
            vel.linear.x = -lin_vel_abs
        elif dt < 6:
            vel.angular.z = ang_vel_abs
        elif dt < 8:
            vel.angular.z = -ang_vel_abs
        else:
            self.__tm = tm
        self.__pub_cmd_vel.publish(vel)


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    cmd_vel_topic = "/cmd_vel"
    odometry_topic = "/odom"
    process_rate = rospy.get_param("~process_rate", 5.0)
    node = TestMotors(cmd_vel_topic,  odometry_topic)
    rate = rospy.Rate(process_rate)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
