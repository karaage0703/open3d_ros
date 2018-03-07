#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

import util
import py3d

def view_pcl(pcl_data):
    py3d.draw_geometries([pcl_data])


def callback(data):
    pcl_data = util.convert_pcl(data)

    view_pcl(pcl_data)

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('input',
                     PointCloud2, callback)

    rospy.spin()
