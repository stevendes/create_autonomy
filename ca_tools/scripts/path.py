#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped


class PathPublisher:
    """
    Class that contains all the functions.
    """
    def __init__(self):
        """
        Initialization of the variables.
        """
        self.odom_sub = rospy.Subscriber('/create1/odom', Odometry, self.odom_cb)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self._path = Path()
        self._pose = PoseStamped()

    def odom_cb(self, data):
        """
        Callback functions that gets information from the Odometry of the robot and
        publish the Path message.
        """
        self._path.header = data.header
        self._pose.header = data.header
        self._pose.pose = data.pose.pose
        self._path.poses.append(self._pose)
        self.path_pub.publish(self._path)


if __name__ == '__main__':
    try:
        rospy.init_node('path_node')
        _path_publisher = PathPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
