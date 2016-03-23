#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Derek King

from __future__ import print_function

import actionlib
from amcl.msg import CheckPosesAction, CheckPosesGoal
from geometry_msgs.msg import Pose
from math import cos, sin, pi
import rospy
import sys

class CheckPoseClient():
    def __init__(self):
        """ Check poses client for 2d pose """
        self.client = actionlib.SimpleActionClient('check_poses/check_poses', CheckPosesAction)

    def check_pose_2d(self, x, y, angle):
        print("Waiting for server..")
        self.client.wait_for_server()

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = sin(angle/2)
        pose.orientation.w = cos(angle/2)
        goal = CheckPosesGoal()
        #goal.poses.poses.append(pose)
        goal.poses.header.stamp = rospy.Time.now()
        goal.poses.header.frame_id = "map"

        self.client.send_goal(goal)

        self.client.wait_for_result()

        result = self.client.get_result()
        if not result.is_valid:
            print("ERROR :", result.message)
        else:
            print("Print result weight is ", result.weights)


def main():
    if len(sys.argv) != 4:
        print("Usage : check_poses_client.py x y angle")
        print("  angle is in degrees")
        sys.exit(1)
    x, y, angle = [float(arg) for arg in sys.argv[1:4]]
    angle *= pi/180.0  # angle is in degrees (radians is hard to use)
    rospy.init_node('check_poses_client',anonymous=True)
    client = CheckPoseClient()
    client.check_pose_2d(x,y,angle)


if __name__ == '__main__':
    main()
