#! /usr/bin/env python

import rospy
import ca_tools.msg
import actionlib


def line_follower_client():

    client = actionlib.SimpleActionClient(
        'linefollower', ca_tools.msg.linefollowerAction)

    client.wait_for_server()   #wait until the server is up

    goal = ca_tools.msg.linefollowerGoal(duration_threshold=5000, times_oop=50) # create a sample goal

    client.send_goal(goal) #send it to the server

    client.wait_for_result() #wait until result is ready

    return client.get_result() #return the result


if __name__ == '__main__':
    try:
        rospy.init_node('lineclient')

        result = line_follower_client()
        rospy.loginfo("THE RESULT WAS : %r , AND THE TOTAL TIME WAS %d",
                      result.result, result.total_time.to_sec())
    except rospy.ROSInterruptException:
        pass
