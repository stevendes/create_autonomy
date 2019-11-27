#! /usr/bin/env python

import smach
import smach_ros
from smach import CBState

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from ca_msgs.msg import Bumper

wall_sensor = False
bumper_sensor = Bumper()


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['continue', 'avoid_wall'])
def move_cb(user_data):
    global wall_sensor, bumper_sensor
    rospy.loginfo('Moving')
    move_topic = rospy.Publisher('/create1/cmd_vel/', Twist, queue_size=1)
    rospy.sleep(0.1)
    msg = Twist()
    msg.linear.x = 0.3
    move_topic.publish(msg)
    if not wall_sensor and not bumper_sensor.is_left_pressed and not bumper_sensor.is_right_pressed:
        return 'continue'
    if (not wall_sensor and (bumper_sensor.is_right_pressed or bumper_sensor.is_left_pressed)) or wall_sensor:
        return 'avoid_wall'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['turn_left', 'turn_right'])
def avoid_wall_cb(user_data):
    global wall_sensor,bumper_sensor
    if wall_sensor or bumper_sensor.is_left_pressed:
        turn = 'left'
    if bumper_sensor.is_right_pressed:
        turn = 'right'
    rospy.loginfo('Rewinding')
    avoid_topic = rospy.Publisher('/create1/cmd_vel/', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = -0.3
    avoid_topic.publish(msg)
    rospy.sleep(2)
    if turn == 'left':
        return 'turn_left'
    if turn == 'right':
        return 'turn_right'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['continue'])
def turn_left_cb(user_data):
    rospy.loginfo('Turning')
    turn_topic = rospy.Publisher('/create1/cmd_vel/', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = -0.2
    turn_topic.publish(msg)
    rospy.sleep(7.5)
    return 'continue'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['continue'])
def turn_right_cb(user_data):
    rospy.loginfo('Turning')
    turn_topic = rospy.Publisher('/create1/cmd_vel/', Twist, queue_size=1)
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.2
    turn_topic.publish(msg)
    rospy.sleep(7.5)
    return 'continue'


def sensor_callback(data):
    global wall_sensor
    wall_sensor = data.data


def bumper_callback(data):
    global bumper_sensor
    bumper_sensor.is_left_pressed = data.is_left_pressed
    bumper_sensor.is_right_pressed = data.is_right_pressed


if __name__ == '__main__':
    try:
        rospy.init_node('key_teleop')

        vwall_subs = rospy.Subscriber("/create1/virtual_wall/",Bool , sensor_callback)
        bumper_subs = rospy.Subscriber("/create1/bumper/",Bumper , bumper_callback)

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['outcome'])

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('robot_server', sm, '/SM_ROBOT')
        sis.start()

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('MOVE', CBState(move_cb),
                                 {'continue': 'MOVE','avoid_wall': 'AVOID_WALL'})
            smach.StateMachine.add('AVOID_WALL', CBState(avoid_wall_cb),
                                   {'turn_left': 'TURN_LEFT', 'turn_right': 'TURN_RIGHT'})
            smach.StateMachine.add('TURN_LEFT', CBState(turn_left_cb),
                                   {'continue': 'MOVE'})
            smach.StateMachine.add('TURN_RIGHT', CBState(turn_right_cb),
                                   {'continue': 'MOVE'})

        # Execute SMACH plan
        outcome = sm.execute()

        sis.stop()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
