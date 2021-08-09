#!/usr/bin/env python

import sys
import os
import rospy
import numpy as np
import cv2
import pcl
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from cv_bridge import CvBridge

save_path = None

def ros_to_pcl(msg):
    point_list = []
    
    for data in pc2.read_points(msg, skip_nans=True):
    points_list.append([data[0], data[1], data[2], data[3]])
    
    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)
    
    return pcl_data
    
def cloud_loader(scan_cb):
    timestamp = msg.header.stamp.secs + ((msg.header.stamp.nsecs + 0.0) / 1000000000)
    save_pcd(scan_cb, timestamp, save_path)

def save_pcd(scan_cb, timestamp, path):
    p = pcl.PointCloud(np.array(list(pc2.read_points(scan_cb)), dtype=np.float32)[:, 0:3])
    p.to_file(path + '/pcd' + '_' + "{:.5f}".format(timestamp) + '.pcd')


def rosbag_data_extract_sample():
    global save_path
    topic="/converted_pc"
    try:
        save_path = sys.argv[1]
        topic = sys.argv[2]
    except Exception, e:
        #sys.exit("Please specify the save path. Example: rosbag_data_extract_unsync.py /media/0/output/")
        save_path = './sample'
    
    node_name = "get_%s_and_convert_to_PCD_data" %topic
    rospy.init_node('rosbag_pcd_extract_unsync', anonymous=True)

    rospy.Subscriber(topic, PointCloud2, cloud_loader)
    rospy.spin()


if __name__ == '__main__':
    rosbag_data_extract_sample()
