#! /usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

CMD_VEL_TOPIC = "/cmd_vel"
GT_TOPIC = "/gts"

class Kinematics(object):
    """docstring for Kinematics."""
    def __init__(self):
        self.pose = Pose()
        self.initial_pose = Pose()
        rospy.init_node("kinematics_node")
        self.vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        self.gt_sub = rospy.Subscriber(GT_TOPIC, Odometry, self._gt_cb)

    def move(self, lin, ang, duration_sec):
        # Generate Twist message
        twist_msg = self._twist(lin, ang)
        # Record initial pose
        self.initial_pose = self.pose
        # Move robot at the specified velocity during 'duration_sec'
        now = rospy.Time.now().secs
        while (rospy.Time.now().secs - now) <= duration_sec:
            self.vel_pub.publish(twist_msg)
        # Stop robot
        self.vel_pub.publish(self._twist(0, 0))
        end_pose = self.pose
        rospy.loginfo("[{0:.3f}, {1:.3f}, {2:.3f}]".format(
            end_pose.position.x - self.initial_pose.position.x,
            end_pose.position.y - self.initial_pose.position.y,
            self._get_quat_diff(end_pose.orientation, self.initial_pose.orientation)
        ))
        rospy.sleep(5.)

    def _gt_cb(self, msg):
        self.pose = msg.pose.pose

    def _get_quat_diff(self, q1, q0):
        diff = self._get_yaw_from_quat(q1) - self._get_yaw_from_quat(q0)
        while diff < -math.pi:
            diff += 2*math.pi
        while diff > math.pi:
            diff -= 2*math.pi
        return math.degrees(diff)

    def _get_yaw_from_quat(self, q):
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        return euler[2]

    def _twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        return msg

def kinematics():
    kine = Kinematics()
    # Move forward
    kine.move(0.2, 0, 5)
    # Move backward
    kine.move(-0.2, 0, 5)
    # Rotate left
    # dT * dAngle = PI
    kine.move(0, 0.5236, 6)
    # Rotate right
    kine.move(0, -0.5236, 6)

if __name__ == '__main__':
    kinematics()
