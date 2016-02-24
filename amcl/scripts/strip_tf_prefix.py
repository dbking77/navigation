#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Derek King

from __future__ import print_function

import rosbag
import sys

def fix_frame_id(frame_id):
    if len(frame_id) and (frame_id[0] == '/'):
        return frame_id[1:]
    return frame_id

def process_bag(in_bagfn, out_bagfn):
    # strip leading frame id's from input bag file, and write result to output bag file
    print("Opening bag", in_bagfn)
    in_bag = rosbag.Bag(in_bagfn, mode='r')
    out_bag = rosbag.Bag(out_bagfn, mode='w', compression='bz2')

    print("Processing bag", in_bagfn)
    for topic, msg, t in in_bag.read_messages():
        if topic in ('tf','/tf'):
            for transform in msg.transforms:
                transform.header.frame_id = fix_frame_id(transform.header.frame_id)
                transform.child_frame_id = fix_frame_id(transform.child_frame_id)
        elif hasattr(msg,'header'):
            msg.header.frame_id = fix_frame_id(msg.header.frame_id)
        out_bag.write(topic, msg, t)

    print("Closing bag", in_bagfn)
    out_bag.close()
    in_bag.close()

def main():
    if len(sys.argv) != 3:
        print("Usage : strip_tf_prefix <in.bag> <out.bag>")
        sys.exit(1)
    process_bag(sys.argv[1], sys.argv[2])

if __name__ == "__main__":
    main()
