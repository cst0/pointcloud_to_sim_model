#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import read_points


def cloud_to_array(cloud):
    points = []
    for point in read_points(cloud, skip_nans=True):
        points.append(point[:3])
    rospy.loginfo(f"Read {len(points)} points from pointcloud")
    points = np.array(points)
    return points


def rgb_from_pointcloud2(cloud):
    points = []
    for point in read_points(cloud, skip_nans=True):
        points.append(point[3:])
    rospy.loginfo(f"Read {len(points)} points from pointcloud")
    points = np.array(points)
    return points


def array_to_cloud(points, name):
    cloud = PointCloud2()
    cloud.header.frame_id = name
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud


def prune_points(points, min_, max_):
    # points is a numpy array of shape (n, 3)
    # if any of the points are outside the min_ and max_ bounds, remove them
    # min_ and max_ are just float values
    # returns a numpy array of shape (m, 3)
    under_zero_count = (
        len(points[points[:, 0] < 0])
        + len(points[points[:, 1] < 0])
        + len(points[points[:, 2] < 0])
    )
    if under_zero_count > 0:
        rospy.logwarn(f"Found {under_zero_count} points under zero")
    len1 = len(points)
    points = points[points[:, 0] > min_]
    points = points[points[:, 1] > min_]
    points = points[points[:, 2] > min_]
    len2 = len(points)
    points = points[points[:, 0] < max_]
    points = points[points[:, 1] < max_]
    points = points[points[:, 2] < max_]
    len3 = len(points)
    rospy.loginfo(
        f"Pruned {len1 - len2} points below threshold, {len2 - len3} points above threshold, {len3} points remaining"
    )
    return points
