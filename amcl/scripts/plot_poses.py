#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Derek King

from __future__ import print_function

from math import pi
import pylab
import rosbag
import sys

t_start = None

def plot_bag(bagfn, color):
    global t_start

    print("Opening bag", bagfn)

    bag = rosbag.Bag(bagfn, mode='r')

    print("Reading bag", bagfn)

    # extract data from bag
    keys = ('t', 'x','y','a','cxx','cyy','caa')
    data = {k:[] for k in keys}
    for topic, msg, t in bag.read_messages(topics='amcl_pose'):
        tm = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        c = msg.pose.covariance
        # c[0] is row 0 col 0 (covariance of x,x position)
        # c[6] is row 1 col 1 (covariance of y,y posision)
        # c[35] is row 5, col5 (covariance of z,z orientation)
        values = (tm, p.x, p.y, o.z, c[0], c[7], c[35])
        for k,v in zip(keys,values):
            data[k].append(v)
    bag.close()

    # convert all list to pylab.arrays
    for k in keys:
        data[k] = pylab.array(data[k])

    t, x, y, a, cxx, cyy, caa = [data[k] for k in keys]

    if t_start is None:
        t_start = t[0]
        print("Setting start time to", t_start)

    t-=t_start

    pylab.figure("path")
    pylab.plot(x,y,color)
    pylab.axis('equal')

    std_r = pylab.sqrt(cxx+cyy)
    pylab.figure("radial std")
    pylab.plot(t, std_r, color)

    std_a = pylab.sqrt(caa)*(180/pi)
    pylab.figure("angular std")
    pylab.plot(t, std_a, color)

    print("  radial std  \tavg=%.3f \tmax=%.3f" % (std_r.mean(), std_r.max()))
    print("  angular std \tavg=%.3f \tmax=%.3f" % (std_a.mean(), std_a.max()))


def main():
    if len(sys.argv) < 2:
        print("usage : plot_poses.py <bag1.fn> [bag2.fn] ...")
        sys.exit(1)

    colors = ['r','b','g','k','y','c']
    for index, bagfn in enumerate(sys.argv[1:]):
        color = colors[index % len(colors)]
        plot_bag(bagfn,color)
    pylab.show()

if __name__ == "__main__":
    main()
