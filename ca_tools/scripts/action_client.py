#! /usr/bin/env python

import rospy
import actionlib
from ca_tools.msg import FollowLineAction, FollowLineGoal
from geometry_msgs.msg import Point


def robot_client():
    _client = actionlib.SimpleActionClient('action_server', FollowLineAction)
    _client.wait_for_server()
    _goal_seconds = rospy.get_param("/get_seconds")
    _goal_position = Point()
    _goal_position.x = rospy.get_param("/get_pos_x")
    _goal_position.y = rospy.get_param("/get_pos_y")
    goal = FollowLineGoal(goal_seconds=_goal_seconds, goal_position=_goal_position)
    _client.send_goal(goal)
    _client.wait_for_result()
    return _client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('action_client')
        result = robot_client()
    except rospy.ROSInterruptException:
        pass
