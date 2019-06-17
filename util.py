#!/usr/bin/env python

# import roslib
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

pub = rospy.Publisher('/output', PointCloud2, queue_size=1)
tmp_pcd_name = "/tmp/tmp_cloud.pcd"

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]

TEST_POINTS = [
    [0.3, 0.0, 0.0, 0xff0000],
    [0.0, 0.3, 0.0, 0x00ff00],
    [0.0, 0.0, 0.3, 0x0000ff],
]

def convert_pcl(data):
    header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH %d
HEIGHT %d
VIEWPOINT 0 0 0 1 0 0 0
POINTS %d
DATA ascii'''

    with open(tmp_pcd_name, 'w') as f:
        f.write(header % (data.width, data.height, data.width*data.height))
        f.write("\n")

        for p in pc2.read_points(data, skip_nans=True):
            f.write('%f %f %f %e' % (p[0], p[1], p[2], p[3]))
            f.write("\n")

        cloud_list = []
        for p in pc2.read_points(data, skip_nans=False):
            cloud_list.append(p[0])
            cloud_list.append(p[1])
            cloud_list.append(p[2])
            cloud_list.append(p[3])

        f.write("\n")

    pcd = o3d.read_point_cloud(tmp_pcd_name)

    return pcd

def publish_pointcloud(output_data, input_data):
    # convert pcl data format
    pc_p = np.asarray(output_data.points)
    pc_c = np.asarray(output_data.colors)
    tmp_c = np.c_[np.zeros(pc_c.shape[1])]
    tmp_c = np.floor(pc_c[:,0] * 255) * 2**16 + np.floor(pc_c[:,1] * 255) * 2**8 + np.floor(pc_c[:,2] * 255) # 16bit shift, 8bit shift, 0bit shift

    pc_pc = np.c_[pc_p, tmp_c]

    # publish point cloud
    output = pc2.create_cloud(Header(frame_id=input_data.header.frame_id), FIELDS , pc_pc)
    pub.publish(output)

def publish_testcloud(input_data):
    # publish point cloud
    output = pc2.create_cloud(Header(frame_id=input_data.header.frame_id), FIELDS , TEST_POINTS)
    pub.publish(output)



def callback(data):
    result_pcl = convert_pcl(data)
    print(result_pcl)

    publish_pointcloud(result_pcl, data)

    # publish_testcloud(data)

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('input',
                     PointCloud2, callback)

    rospy.spin()
