#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.is_red_pub = rospy.Publisher("isred",Bool,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/create1/raspicam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    low_red = np.array([ 161, 155, 84])

    high_red = np.array([ 179, 255, 255])

    mask = cv2.inRange(hsv_frame,low_red,high_red)
    (rows,cols,_) = cv_image.shape
    is_red = Bool()
    is_red.data = (cv2.countNonZero(mask) / float(rows * cols) > 0.018)
    self.is_red_pub.publish(is_red)

    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)