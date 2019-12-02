#! /usr/bin/env python

import rospy
import actionlib
from ca_tools.msg import FollowLineAction, FollowLineGoal
from geometry_msgs.msg import Point


def robot_client():
    client = actionlib.SimpleActionClient('follow_line_action', FollowLineAction)
    client.wait_for_server()
    goal_secs = rospy.get_param("/get_seconds")
    goal_pose = Point()
    goal_pose.x = rospy.get_param("/get_pos_x")
    goal_pose.y = rospy.get_param("/get_pos_y")
    goal = FollowLineGoal(goal_seconds=goal_secs, goal_position=goal_pose)
    client.send_goal(goal)
    client.wait_for_result()
    return _client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('follow_line_client')
        result = robot_client()
    except rospy.ROSInterruptException:
        pass
