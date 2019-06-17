#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

import util
import open3d as o3d
import numpy as np

def numpy_pcl(pcl_data):
    tmp_pcl = pcl_data

    pc_p = np.asarray(pcl_data.points)
    pc_c = np.asarray(pcl_data.colors)

    pc_c[:,0] = 0

    pc_c3d = o3d.Vector3dVector(pc_c)

    tmp_pcl.colors = pc_c3d

    return tmp_pcl

def callback(data):
    pcl_data = util.convert_pcl(data)

    result_pcl = numpy_pcl(pcl_data)

    util.publish_pointcloud(result_pcl, data)

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('input',
                     PointCloud2, callback)

    rospy.spin()
