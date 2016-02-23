#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Derek King

from __future__ import print_function

from math import pi
import pylab
import rosbag
import sys

t_start = None


def read_bag(bagfn):
    print("Opening bag", bagfn)

    bag = rosbag.Bag(bagfn, mode='r')

    print("Reading bag", bagfn)

    # extract data from bag
    keys = ('t','x','y','a','cxx','cyy','caa')
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

    return data


def compare_bags(bagfn1, bagfn2):
    """ Compares bagfn1 against bagfn2 """
    d1 = read_bag(bagfn1)
    d2 = read_bag(bagfn2)
    t1 = d1['t']
    t2 = d2['t']

    # TODO make sure times are increasing
    if pylab.any(t1 != pylab.sort(t1)):
        raise RuntimeError("Bag message times are not in order", bagfn1)
    if pylab.any(t2 != pylab.sort(t2)):
        raise RuntimeError("Bag message times are not in order", bagfn2)

    tmin = min(t1[0], t2[0])
    tmax = max(t1[-1], t2[-1])
    dt = 0.1
    ts = pylab.arange(tmin,tmax,dt)

    times = [t1, t2, ts]

    for t in times:
        x1 = pylab.interp(t, t1, d1['x'])
        x2 = pylab.interp(t, t2, d2['x'])
        y1 = pylab.interp(t, t1, d1['y'])
        y2 = pylab.interp(t, t2, d2['y'])
        a1 = pylab.interp(t, t1, d1['a'])
        a2 = pylab.interp(t, t2, d2['a'])

        dr = pylab.sqrt((x1-x2)**2 + (y1-y2)**2).mean()
        da = abs(a1-a2).mean()
        print("Compart dr=%0.3f cm da=%0.1f degress" % (dr*1e2,da*180/pi))
    

def plot_bag(bagfn, color):
    global t_start
    data = read_bag(bagfn)

    keys = ('t','x','y','a','cxx','cyy','caa')
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
    if len(sys.argv) < 3:
        print("usage : plot_poses.py plot <bag1.fn> [bag2.fn] ...")
        print("usage : plot_poses.py compare <good1.fn> <bag2.fn>")
        sys.exit(1)

    if sys.argv[1] == 'plot':
        colors = ['r','b','g','k','y','c']
        for index, bagfn in enumerate(sys.argv[2:]):
            color = colors[index % len(colors)]
            plot_bag(bagfn,color)
        pylab.show()
    elif sys.argv[1] == 'compare':
        compare_bags(sys.argv[2], sys.argv[3])
    else:
        print("Invalid option", sys.argv[1])

if __name__ == "__main__":
    main()
