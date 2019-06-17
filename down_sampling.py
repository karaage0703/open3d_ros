#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

import util
import open3d as o3d

def down_sampling(pcl_data):
    downpcd = o3d.voxel_down_sample(pcl_data, voxel_size = 0.05)

    return downpcd


def callback(data):
    pcl_data = util.convert_pcl(data)

    result_pcl = down_sampling(pcl_data)

    util.publish_pointcloud(result_pcl, data)

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('input',
                     PointCloud2, callback)

    rospy.spin()
