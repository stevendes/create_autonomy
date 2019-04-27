#! /usr/bin/env python

import math
import rospy
import threading
import PyKDL

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties

CMD_VEL_TOPIC = "/cmd_vel"
GT_TOPIC = "/gts"

class Kinematics(object):
    """docstring for Kinematics."""
    def __init__(self):
        self.pose = Pose()
        self.initial_pose = Pose()
        self.pose_lock = threading.Lock()
        rospy.init_node("kinematics_node")
        self.vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        self.gt_sub = rospy.Subscriber(GT_TOPIC, Odometry, self._gt_cb)
        # Increase RTF of the simulation to speed it up
        self._set_rtf(2000.)
        # This sleep avoids issues with the first case
        rospy.sleep(rospy.Duration(secs=2))

    def move(self, lin, ang, duration_sec):
        # Generate Twist message
        twist_msg = self._twist(lin, ang)
        # Record initial pose
        self.pose_lock.acquire()
        self.initial_pose = self.pose
        self.pose_lock.release()
        ######
        angle = self._get_yaw_from_quat(self.initial_pose.orientation)
        rospy.loginfo("INITIAL: {}".format(math.degrees(angle)))
        ######
        # Move robot at the specified velocity during 'duration_sec'
        now = rospy.Time.now().secs
        while (rospy.Time.now().secs - now) <= duration_sec:
            self.vel_pub.publish(twist_msg)
        # Stop robot
        self.vel_pub.publish(self._twist(0, 0))
        rospy.sleep(rospy.Duration(nsecs=2000))
        self.pose_lock.acquire()
        end_pose = self.pose
        self.pose_lock.release()
        # Get delta angle
        ######
        angle = self._get_yaw_from_quat(end_pose.orientation)
        rospy.loginfo("END POSE: {}".format(math.degrees(angle)))
        ######
        angle = self._get_quat_diff(end_pose.orientation, self.initial_pose.orientation)
        rospy.loginfo("[{0:.3f}, {1:.3f}, {2:.3f}]".format(
            abs(end_pose.position.x - self.initial_pose.position.x),
            abs(end_pose.position.y - self.initial_pose.position.y),
            math.degrees(angle)
        ))
        ######
        rospy.loginfo("DIFF: {}".format(math.degrees(angle)))
        ######
        rospy.sleep(rospy.Duration(secs=5))

    def _gt_cb(self, msg):
        self.pose_lock.acquire()
        self.pose = msg.pose.pose
        self.pose_lock.release()

    def _get_quat_diff(self, q1, q0):
        TWO_PI = 2. * math.pi
        angle1 = self._get_yaw_from_quat(q1)
        angle0 = self._get_yaw_from_quat(q0)
        diff = angle1 - angle0
        rospy.loginfo("> diff: {}".format(diff))
        if diff >= TWO_PI:
          diff -= TWO_PI
        if diff < 0:
          diff += TWO_PI
        return diff

    def _get_yaw_from_quat(self, q):
        q_arr = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(q_arr)
        return euler[2]

    def _twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        return msg
    
    def _set_rtf(self, rtf):
        rospy.wait_for_service('gazebo/set_physics_properties')
        rospy.wait_for_service('gazebo/get_physics_properties')
        get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
        physics_properties = get_physics_properties_prox()
        physics_properties.max_update_rate = rtf
        rospy.loginfo("Setting RTF to {0}".format(rtf))
        set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
        set_physics_properties_prox(physics_properties.time_step,
                                    physics_properties.max_update_rate,
                                    physics_properties.gravity,
                                    physics_properties.ode_config)

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
