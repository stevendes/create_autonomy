#! /usr/bin/env python

import math
import rospy
import threading
import PyKDL

from geometry_msgs.msg import Twist, Pose, Pose2D, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties

CMD_VEL_TOPIC = "/cmd_vel"
GT_TOPIC = "/gts"

class Kinematics(object):
    """docstring for Kinematics."""
    def __init__(self):
        self.pose = Pose2D()
        self.last_pose = None
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
        self.pose = Pose2D()
        self.pose_lock.release()
        # Move robot at the specified velocity during 'duration_sec'
        now = rospy.Time.now().secs
        while (rospy.Time.now().secs - now) <= duration_sec:
            self.vel_pub.publish(twist_msg)
        # Stop robot
        self.vel_pub.publish(self._twist(0, 0))
        rospy.sleep(rospy.Duration(nsecs=2000))
        # Get final measurements
        self.pose_lock.acquire()
        rospy.loginfo("[{0:.3f}, {1:.3f}, {2:.3f}]".format(
            self.pose.x,
            self.pose.y,
            math.degrees(self.pose.theta)
        ))
        self.pose_lock.release()
        rospy.sleep(rospy.Duration(secs=5))

    def _gt_cb(self, msg):
        self.pose_lock.acquire()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        self.pose_lock.release()
        if(self.last_pose is None):
            # First measurement
            self.last_pose = Pose2D()
            self.last_pose.x = x
            self.last_pose.y = y
            self.last_pose.theta = self._get_yaw_from_quat(o)
        else:
            # Other measurements
            self.pose.x += (x - self.last_pose.x)
            self.pose.y += (y - self.last_pose.y)
            [qx, qy, qz, qw] = quaternion_from_euler(0, 0, self.last_pose.theta)
            angle_diff = self._get_quat_diff(o, Quaternion(qx, qy, qz, qw))
            self.pose.theta += angle_diff
            # Update last pose
            self.last_pose.x = x
            self.last_pose.y = y
            self.last_pose.theta = self._get_yaw_from_quat(o)

    def _get_quat_diff(self, q1, q0):
        angle1 = PyKDL.Rotation.Quaternion(q1.x, q1.y, q1.z, q1.w)
        angle0 = PyKDL.Rotation.Quaternion(q0.x, q0.y, q0.z, q0.w)
        rot = angle1 * angle0.Inverse()
        # get the RPY (fixed axis) from the rotation
        [_, _, yaw] = rot.GetRPY()
        return yaw

    def _get_yaw_from_quat(self, q):
        q_arr = [q.x, q.y, q.z, q.w]
        [_, _, yaw] = euler_from_quaternion(q_arr)
        return yaw

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
