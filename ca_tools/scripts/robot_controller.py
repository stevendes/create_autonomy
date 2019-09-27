#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class RobotController:
    """
    Class to control the Create Robot, had the const of velocity as a parameters
    """
    _FORWARD_VEL = 0.28
    _ANGULAR_VEL = 0.32

    def __init__(self):
        """
        Initialization of the Class, have the publisher and subscribers for /gts and the color sensors, and some
        extra params like
        :param stuck_timer : a timer that is started when the robot is not moving forward
        :param stuck_flag : a flag that is set when the robot is not moving forward
        """
        self.twist_publisher = rospy.Publisher('/create1/cmd_vel', Twist, queue_size=10)

        self.left_sensor_subscriber = rospy.Subscriber(
            "/color_sensor_plugin/left_sensor", Bool, self.callback_left)
        self.right_sensor_subscriber = rospy.Subscriber(
            "/color_sensor_plugin/right_sensor", Bool, self.callback_right)

        self.left_sensor_data = False
        self.right_sensor_data = False

        self.stuck_timer = rospy.Time.now()
        self.stuck_flag = False

        self.r = rospy.Rate(50)

    def callback_left(self,data):
        """
        A callback to get data form the left sensor
        """
        self.left_sensor_data = data.data

    def callback_right(self,data):
        """
        A callback to get data form the right sensor
        """
        self.right_sensor_data = data.data

    def publish_velocity(self, _linear=0, _angular=0):
        """
        Method used to publsih the velocity message and move the robot
        :param _linear: gets
        :param _angular:
        :return:
        """
        _vel = Twist()
        _vel.linear.x = _linear
        _vel.angular.z = _angular
        self.twist_publisher.publish(_vel)

    def move_forward(self):
        """
        Method to move the robot forward
        """
        self.publish_velocity(_linear=self._FORWARD_VEL)

    def move_left(self):
        """
        Method to move to the left
        """
        self.publish_velocity(_angular=self._ANGULAR_VEL)

    def move_right(self):
        """
        Method to move to the right
        """
        self.publish_velocity(_angular=-self._ANGULAR_VEL)

    def solve_stuck(self):
        """
        Method that's get called when the 5 seconds passed and the robot still didn't move forward,
        it forces a forward move
        """
        self.move_forward()
        self.stuck_flag = False
        rospy.sleep(0.1)

    def control (self):
        """
        A method that checks the status of the right and left sensors and based on that call the
        moving methods
        """
        if self.left_sensor_data and self.right_sensor_data:
            self.move_forward()
            self.stuck_flag = False

        if not self.left_sensor_data and self.right_sensor_data:
            self.move_left()
            if not self.stuck_flag:
                self.stuck_timer = rospy.Time.now()
                self.stuck_flag = True
            if rospy.Time.now() - self.stuck_timer > rospy.Duration(5, 0) and self.stuck_flag:
                self.solve_stuck()

        if self.left_sensor_data and not self.right_sensor_data :
            self.move_right()
            if not self.stuck_flag:
                self.stuck_timer = rospy.Time.now()
                self.stuck_flag = True
            if rospy.Time.now() - self.stuck_timer > rospy.Duration(5, 0) and self.stuck_flag:
                self.solve_stuck()

    def execution(self):
        """
        A method that loops the control method in order to constantly check
        """
        while not rospy.is_shutdown():
            self.control()
            self.r.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('control_robot')
        robot = RobotController()
        robot.execution()
    except rospy.ROSInterruptException:
        pass
