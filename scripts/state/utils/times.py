# -*- coding: utf-8 -*-
import rospy


class TimeChecker(object):
    def __init__(self, timeout=None):
        self.timeout = timeout
        self.start_time = None
        self.end_time = None
        self.tm = None

    def start(self):
        self.start_time = rospy.get_time()
        self.tm = self.start_time
        if self.timeout is not None:
            self.end_time = self.start_time + self.timeout

    def delta_time(self):
        tm = rospy.get_time()
        delta = tm - self.tm
        self.tm = tm
        return delta

    def is_timeout(self):
        if self.end_time is not None:
            return rospy.get_time() >= self.end_time
        return False

    def elapsed(self, tm=None):
        if tm is None:
            return rospy.get_time() - self.start_time
        return tm - self.start_time
