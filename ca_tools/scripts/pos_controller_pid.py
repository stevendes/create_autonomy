#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from tf.transformations import euler_from_quaternion


class DrawSquare:
    """
    Class that contains all the functions of the program.
    """
    def __init__(self):
        """
        Initialization of the variables that are going to be used in the class.
        """

        #ROS variables

        self.twist_publisher = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber("/create1/gts", Odometry, self.callback)

        # Time variables

        self._now = 0.0
        self._time_change = 0.0
        self._last_time = 0.0

        # Angle variables

        self._current_angle = 0.0
        self._set_angle = 0.0
        self._error_angle = 0.0
        self._err_sum_angle = 0.0
        self._control_error_angle = 0.01

        # Linear movement variables

        self._set_linear_goal = Pose2D()
        self._error_length = 0.0
        self._err_sum_length = 0.0
        self._control_error_length = 0.1
        self._curr_aux = Pose2D()

        # State Variables

        self._set_variable = True
        self._state = "FORWARD"

        # Odometry Flag

        self._odom_flag = False

        # PID variables
        self._kp = 5
        self._ki = 2
        self._kd = 0

        # Range of output

        self._max_angular_vel = 0.4
        self._max_linear_vel = 0.4

    def callback(self, data):
        """
        Callback function to update the pose according to the robot's movement.
        """

        q = (data.pose.pose.orientation.x,
             data.pose.pose.orientation.y,
             data.pose.pose.orientation.z,
             data.pose.pose.orientation.w)

        _, _, self._current_angle = euler_from_quaternion(q)

        self._curr_aux.x = data.pose.pose.position.x
        self._curr_aux.y = data.pose.pose.position.y

        self._odom_flag = True

    def get_time(self):
        """
        Function to get the current time.
        """

        self._now = rospy.get_time()
        if self._last_time < 0.001:
            self._last_time = self._now
        self._time_change = self._now - self._last_time

    def update_time(self):
        """
        Function to update the Last Time.
        """

        if self._now > 0:
            self._last_time = self._now

    def set_goals(self):
        """
        Function that sets the new goals checking the state and if the _set_variable flag
        is turned True, if it is true, means that a new goal needs to be set.
        """

        _ini_aux = Pose2D()

        if self._state == "FORWARD" and self._set_variable:
            _input_angle = self._current_angle
            _ini_aux.x = self._curr_aux.x
            _ini_aux.y = self._curr_aux.y
            self._set_linear_goal.x = _ini_aux.x + 2 * math.cos(_input_angle)
            self._set_linear_goal.y = _ini_aux.y + 2 * math.sin(_input_angle)
            self._set_angle = _input_angle
            self._set_variable = False

        if self._state == "TURN" and self._set_variable:
            _input_linear_pos = _ini_aux
            _input_angle = self._current_angle
            self._set_linear_goal = _input_linear_pos
            self._set_angle = _input_angle + math.pi / 2
            self._set_angle = math.atan2(math.sin(self._set_angle), math.cos(self._set_angle))
            self._set_variable = False

    def pid(self):
        """
        Function that calculates the error and publish the velocity constantly.
        """

        _is_new_goal = Pose2D()
        _vel = Twist()

        # Error calculations

        self._error_angle = self._set_angle - self._current_angle
        self._err_sum_angle += self._error_angle * self._time_change
        _is_new_goal.x = self._set_linear_goal.x - self._curr_aux.x
        _is_new_goal.y = self._set_linear_goal.y - self._curr_aux.y
        self._error_length = math.hypot(_is_new_goal.x, _is_new_goal.y)
        self._err_sum_length += self._err_sum_length * self._time_change

        if self._state == "FORWARD":
            _output_linear_vel = self._kp * self._error_length + self._ki * self._err_sum_length
            _output_angular_vel = 0.0

        if self._state == "TURN":
            _output_angular_vel = self._kp * self._error_angle + self._ki * self._err_sum_angle
            _output_linear_vel = 0.0

        if _output_linear_vel > self._max_linear_vel:
            _output_linear_vel = self._max_linear_vel
        if _output_linear_vel < 0:
            _output_linear_vel = self._max_linear_vel
        if _output_angular_vel > self._max_angular_vel or _output_angular_vel < 0:
            _output_angular_vel = self._max_angular_vel

        _vel.linear.x = _output_linear_vel
        _vel.angular.z = _output_angular_vel
        self.twist_publisher.publish(_vel)

    def control(self):
        """
        Function that controls if the goal is reached, if it is, change the _set_variable
        flag to True so the set_goals function knows that a new goal need to be set.
        """

        if self._state == "FORWARD" and not self._set_variable:
            if abs(self._error_length) < self._control_error_length:
                self._state = "TURN"
                self._set_variable = True

        if self._state == "TURN" and not self._set_variable:
            if abs(self._error_angle) < self._control_error_angle:
                self._state = "FORWARD"
                self._set_variable = True

    def execution(self):
        """
        Main loop of the program.
        """
        while not rospy.is_shutdown():
            if robot._odom_flag:
                robot.get_time()
                robot.set_goals()
                robot.pid()
                robot.control()
                robot.update_time()
            else:
                pass
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('control_robot')
        robot = DrawSquare()
        robot.execution()
    except rospy.ROSInterruptException:
        pass
