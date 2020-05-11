#!/usr/bin/env python
import rospy

import message_filters
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry

def test(msg):
  print('asdf')

def callback(mode, penalty):
  # The callback processing the pairs of numbers that arrived at approximately the same time
  print('callback')

rospy.init_node('test')
mode_sub = message_filters.Subscriber('/camera/color/image_raw/compressed', CompressedImage)
penalty_sub = message_filters.Subscriber('/Odometry/wheel', Odometry)
# odom_sub = rospy.Subscriber('/Odometry/gps', Odometry, test, queue_size=1)

ts = message_filters.ApproximateTimeSynchronizer([mode_sub, penalty_sub], 10, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.spin()
