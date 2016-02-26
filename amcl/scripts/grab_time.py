#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Derek King

from __future__ import print_function

import rospy


def main():
    rospy.init_node('grab_time.py', anonymous=True)
    while not rospy.is_shutdown():
        raw_input('press enter to get time')
        t = rospy.Time.now()
        print(t.to_sec())

if __name__ == "__main__":
    main()
