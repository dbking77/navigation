#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Derek King

from __future__ import print_function

from math import pi, sin, cos
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# header: 
#   seq: 1
#   stamp: 
#     secs: 1456375512
#     nsecs: 127975773
#   frame_id: map
# pose: 
#   pose: 
#     position: 
#       x: 1.98853218555
#       y: 0.0343072712421
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: 0.707106781187
#       w: 0.707106781187
#   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

def main():
    angle = 90*(pi/180)
    rospy.init_node('initialpose', anonymous=True)
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 2
    msg.pose.pose.position.y = 0
    msg.pose.pose.position.z = 0
    msg.pose.pose.orientation.x = 0
    msg.pose.pose.orientation.y = 0
    msg.pose.pose.orientation.z = sin(angle/2)
    msg.pose.pose.orientation.w = cos(angle/2)
    msg.pose.covariance = [0.0 for i in range(36)]
    msg.pose.covariance[6*0+0] = 0.005
    msg.pose.covariance[6*1+1] = 0.005
    msg.pose.covariance[6*5+5] = 0.005
    raw_input("press enter to publish")
    pub.publish(msg)
    print("published")
    rospy.sleep(1)

if __name__ == "__main__":
    main()
