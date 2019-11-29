#! /usr/bin/env python

import rospy
import math
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ca_tools.msg import FollowLineAction, FollowLineFeedback, FollowLineResult


class RobotAction(object):
    """
    This class keeps track of the goals, check if the position goal is achieved, keep track of the total time and traveled distance since the 
    start of the process
    """
    _ERROR_TOLERANCE = 0.1

    def __init__(self, name):
        self._accumulated_dist = 0.0
        self._current_pos = Point()
        self._last_pos = Point()
        self._gts_first = True
        self._current_pos.x = None
        self._current_pos.y = None
        self._gts_subscriber = rospy.Subscriber("/create1/gts", Odometry, self.gts_callback)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, FollowLineAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._goal_reached = False
#        self._curr_time = rospy.Time.now()
        self._rate = rospy.Rate(2)

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
            self._accumulated_dist += math.hypot(abs( self._last_pos.x - self._current_pos.x), self._last_pos.y -
                                     self._current_pos.y)

    def goal_reach_control(self, goal_point):
        x_limit = (self._current_pos.x <= (goal_point.x + self._ERROR_TOLERANCE)) and (self._current_pos.x
        >= (goal_point.x - self._ERROR_TOLERANCE))
        y_limit = (self._current_pos.y <= (goal_point.y + self._ERROR_TOLERANCE)) and (self._current_pos.y
        >= (goal_point.y - self._ERROR_TOLERANCE))
        return x_limit and y_limit

    def execute_cb(self, goal):
        # helper variables
        success = True
        goal_point = Point()
        goal_point = goal.goal_position
        goal_seconds = goal.goal_seconds
        init_time = rospy.Time.now()
        feedback = FollowLineFeedback()
        result = FollowLineResult()

        while not self._goal_reached:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            curr_time = rospy.Time.now() - init_time
            feedback.accumulated_distance = self._accumulated_dist
            feedback.current_mission_duration = curr_time
            self._as.publish_feedback(feedback)
            rospy.loginfo(feedback.accumulated_distance)
            if self.goal_reach_control(goal_point):
                self._goal_reached = True
            self._rate.sleep()

        if success:
            if curr_time.secs < goal_seconds:
                result.goal_achieved = True
            else:
                result.goal_achieved = False
            result.total_time = curr_time
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(result)


if __name__ == '__main__':
    try:
        rospy.init_node('follow_line_action')
        server = RobotAction(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
