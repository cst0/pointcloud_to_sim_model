#!/usr/bin/env python3
from typing import Tuple, Union

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse

from pointcloud2simmodel.utils import (
    cloud_to_array,
    prune_points,
)

from pointcloud2simmodel.voxelgrid import VoxelGrid
from pointcloud2simmodel.generator import SdfStlGenerator
from pointcloud2simmodel.spawner import PybulletSpawner

class PointCloud2Collector:
    def __init__(self, topic_name) -> None:
        self.topic_name = topic_name
        self.cloud = None
        self.sub = rospy.Subscriber(self.topic_name, PointCloud2, self.callback)
        self.srv = rospy.Service(
            f"{self.topic_name}_to_sdf", Trigger, self.service_callback
        )

    def callback(self, cloud) -> None:
        self.cloud = cloud

    def service_callback(self, _) -> TriggerResponse:
        if self.cloud is None:
            return TriggerResponse(False, "No pointcloud received yet")

        # if any of the dimensions are greater than 10 meters, or closer than
        # 10cm, there's no way that's real... we'll just prune that out
        points = cloud_to_array(self.cloud)
        points = prune_points(
            points, 0.1, 10
        )  # TODO: allow setting these values via rosparam

        vg = VoxelGrid()
        vg.from_pointcloud2(self.cloud)
        # voxels = vg.segment()
        voxels = [vg]

        rospy.loginfo(f"Found {len(voxels)} voxelgrids, displaying all now")
        for v in voxels:
            rospy.loginfo(f"Voxel: {v}")
            # generator = SdfGenerator(v, show_progress=True)
            generator = SdfStlGenerator(v)  # TODO: allow selection via parameter
            generator.write()
            spawner = PybulletSpawner()
            spawner.spawn(generator)

        return TriggerResponse(True, "SDF generated and spawned")

    def get_cloud_dims(
        self, cloud: Union[PointCloud2, np.ndarray]
    ) -> Tuple[float, float, float, float, float, float]:
        if isinstance(cloud, PointCloud2):
            points = cloud_to_array(cloud)
        else:
            points = cloud

        x_min = np.min(points[:, 0])  # type: ignore
        y_min = np.min(points[:, 1])  # type: ignore
        z_min = np.min(points[:, 2])  # type: ignore
        x_max = np.max(points[:, 0])  # type: ignore
        y_max = np.max(points[:, 1])  # type: ignore
        z_max = np.max(points[:, 2])  # type: ignore
        return x_min, y_min, z_min, x_max, y_max, z_max
