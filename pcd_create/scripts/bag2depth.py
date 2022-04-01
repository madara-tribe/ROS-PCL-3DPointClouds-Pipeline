#!/usr/bin/env python3

import os
import argparse
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
H = 960
W = 1280

def arg_parsar():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag_file', type=str, default="/home/parallels/place/depth.bag")
    parser.add_argument('--depth_topic', type=str, default="/camera/depth/image_rect_raw")
    parser.add_argument('--image_topic', type=str, default="/camera/color/image_raw")
    parser.add_argument('--output_path', type=str, default="/home/parallels/place")
    args = parser.parse_args()
    return args

def main():
    args = arg_parsar()
    out_rgb_dir = os.path.join(args.output_path, "rgb")
    out_dep_dir = os.path.join(args.output_path, "depth")
    out_jet_dir = os.path.join(args.output_path, "jet")
    if not os.path.isdir(out_rgb_dir):
        os.makedirs(out_rgb_dir)
    if not os.path.isdir(out_dep_dir):
        os.makedirs(out_dep_dir)
    if not os.path.isdir(out_jet_dir):
        os.makedirs(out_jet_dir)
    rgb_topic = args.image_topic
    depth_topic = args.depth_topic

    bag = rosbag.Bag(args.bag_file, "r")
    topics = bag.get_type_and_topic_info()[1].keys()
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[rgb_topic, depth_topic]):
        if topic==rgb_topic:
            cvimg = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cvimg = cv2.resize(cvimg, (W, H), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(os.path.join(out_rgb_dir, "frame%06i.jpg" % count), cvimg)
            print('rgb', cvimg.shape, cvimg.max(), cvimg.min())
        elif topic==depth_topic:
            cvimg = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cvimg = cv2.resize(cvimg, (W, H), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(os.path.join(out_dep_dir, "frame%06i.png" % count), cvimg)
            
            jet_img = cv2.applyColorMap(cv2.convertScaleAbs(cvimg, alpha=0.08), cv2.COLORMAP_JET)
            cv2.imwrite(os.path.join(out_jet_dir, "frame%06i.png" % count), jet_img)
            print('depth', cvimg.shape, cvimg.max(), cvimg.min())
        print("Wrote image %i" % count)
        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
