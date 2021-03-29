#!/usr/bin/env python3

import numpy as np
import yaml
import tensorflow as tf
import open3d.ml as _ml3d
import open3d
import random
import rospy
import sys, time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

import argparse
import copy
import os
import os.path as osp
import pprint
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from tqdm import tqdm


class pointcloud_segmenter:
    @staticmethod
    def create_pointcloud2(points, labels, stamp=None, frame_id=None):
        """Create a sensor_msgs.PointCloud2 from an array of points and class labels"""

        pointcloud = np.array(np.column_stack([points, labels]))

        header = Header()
        if stamp is None:
            header.stamp = rospy.Time().now()
        else:
            header.stamp = stamp

        if frame_id is None:
            header.frame_id = "world"
        else:
            header.frame_id = frame_id

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("label", 12, PointField.FLOAT32, 1),
        ]

        pc2 = point_cloud2.create_cloud(header, fields, pointcloud)
        return pc2

    @staticmethod
    def create_random_pointlcloud():
        source = [0, 0, 0]
        deviationFromPoint = 10
        num_points = 100

        points_array = []
        labels = np.random.randint(1, 5, num_points)

        for _ in range(num_points):
            newPoint = [source[i] + random.random() * deviationFromPoint for i in range(3)]
            points_array.append(newPoint)

        points = np.array(points_array)

        print(points.shape)
        print(labels.shape)
        return points, labels

    def ros2open3d(self, ros_cloud):
        field_names = [field.name for field in ros_cloud.fields]
        cloud_data = list(point_cloud2.read_points(ros_cloud, skip_nans=False, field_names=field_names))

        open3d_cloud = open3d.geometry.PointCloud()

        xyz = [(x, y, z) for x, y, z, i in cloud_data]
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

        open3d.visualization.draw_geometries([open3d_cloud])

        return open3d_cloud

    def callback(self, data):
        self.ros2open3d(data)

    def __init__(self):
        """Initialize ros publisher, ros subscriber"""

        self.point_pub = rospy.Publisher("/pointcloud_segmented", PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber("/pointcloud_input", PointCloud2, self.callback, queue_size=1)


def main(args):
    """Initializes and cleanup ros node"""

    ps = pointcloud_segmenter()
    rospy.init_node("pointcloud_segmenter", anonymous=False)
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
