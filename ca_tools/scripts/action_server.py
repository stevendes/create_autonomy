#! /usr/bin/env python

import rospy
import math
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ca_tools.msg import FollowLineAction, FollowLineFeedback, FollowLineResult


class RobotAction(object):
    _ERROR_TOLERANCE = 0.1

    def __init__(self, name):
        self._curr_dist = 0.0
        self._current_pos = Point()
        self._last_pos = Point()
        self._gts_first = True
        self._feedback = FollowLineFeedback()
        self._result = FollowLineResult()
        self._gts_subscriber = rospy.Subscriber("/create1/gts", Odometry, self.gts_callback)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, FollowLineAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._goal_reached = False
        self._curr_time = rospy.Time.now()
        self._r = rospy.Rate(2)


    def gts_callback(self, data):
        if self._gts_first:
            self._current_pos.x = data.pose.pose.position.x
            self._current_pos.y = data.pose.pose.position.y
            self._gts_first = False
        else:
            self._last_pos.x = self._current_pos.x
            self._last_pos.y = self._current_pos.y
            self._current_pos.x = data.pose.pose.position.x
            self._current_pos.y = data.pose.pose.position.y
            self._curr_dist += math.hypot(abs( self._last_pos.x - self._current_pos.x), self._last_pos.y -
                                     self._current_pos.y)

    def goal_reach_control(self,_goal_point):
        _x_limit = (self._current_pos.x <= (_goal_point.x + self._ERROR_TOLERANCE)) and (self._current_pos.x
        >= (_goal_point.x - self._ERROR_TOLERANCE))
        _y_limit = (self._current_pos.y <= (_goal_point.y + self._ERROR_TOLERANCE)) and (self._current_pos.y
        >= (_goal_point.y - self._ERROR_TOLERANCE))
        return _x_limit and _y_limit


    def execute_cb(self, goal):
        # helper variables
        success = True
        _goal_point = Point()
        _goal_point = goal.goal_position
        _goal_seconds = goal.goal_seconds
        _init_time = rospy.Time.now()

        while not self._goal_reached:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._curr_time = rospy.Time.now() - _init_time
            self._feedback.accumulated_distance = self._curr_dist
            self._feedback.current_time = self._curr_time
            self._as.publish_feedback(self._feedback)
            rospy.loginfo(self._feedback.accumulated_distance)
            if self.goal_reach_control(_goal_point):
                self._goal_reached = True
            self._r.sleep()

        if success:
            if self._curr_time.secs < _goal_seconds:
                self._result.goal_achieved = True
            else:
                self._result.goal_achieved = False
            self._result.total_time = self._curr_time
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    try:
        rospy.init_node('action_server')
        server = RobotAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
