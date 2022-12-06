#!/usr/bin/env python3
from typing import List, Tuple, Union

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

from pointcloud2simmodel.utils import cloud_to_array

# TODO: allow setting these values via rosparam
DEFAULT_STEP_SIZE = 0.005 # 1cm/2
assert DEFAULT_STEP_SIZE > 0
MAX_SEGMENT_SIZE = 100000


class VoxelGrid:
    def __init__(
        self,
        xdim: float = 0,
        ydim: float = 0,
        zdim: float = 0,
        step: float = DEFAULT_STEP_SIZE,
    ) -> None:

        if xdim == 0 or ydim == 0 or zdim == 0:
            self.xdim = 0
            self.ydim = 0
            self.zdim = 0
            self.step = step
            self.grid = np.array([])
            return
        else:
            self.resize(xdim, ydim, zdim, step)
        self.offset = (
            0,
            0,
            0,
        )  # we're going to be shifting our cloud to be within the smallest possible bounds.

    def __str__(self) -> str:
        return self.__repr__()

    def __repr__(self) -> str:
        return "VoxelGrid(xdim={}, ydim={}, zdim={}, step={}, x_indices={}, y_indices={}, z_indices={}, total cells={}, filled cells={})".format(
            self.xdim,
            self.ydim,
            self.zdim,
            self.step,
            self.x_steps,
            self.y_steps,
            self.z_steps,
            self.x_steps * self.y_steps * self.z_steps,
            np.count_nonzero(self.grid),
        )

    def resize(
        self, xdim: float, ydim: float, zdim: float, step: float = DEFAULT_STEP_SIZE
    ) -> None:
        self.xdim = xdim
        self.ydim = ydim
        self.zdim = zdim
        self.step = step

        self.x_steps = int(xdim / step) + 1
        self.y_steps = int(ydim / step) + 1
        self.z_steps = int(zdim / step) + 1

        rospy.loginfo(
            "Creating voxel grid of size %d x %d x %d",
            self.x_steps,
            self.y_steps,
            self.z_steps,
        )

        self.grid = np.full(
            (self.x_steps, self.y_steps, self.z_steps), False, dtype=bool
        )
        #self.colors = np.full(
        #    (self.x_steps, self.y_steps, self.z_steps, 3), 0, dtype=int
        #)
        rospy.loginfo("Voxel grid created")

    def volume(self) -> int:
        return np.count_nonzero(self.grid) * self.step ** 3

    def populate_point(self, x: float, y: float, z: float, color=(0, 0, 0)) -> None:
        x_index = int(x / self.step)
        y_index = int(y / self.step)
        z_index = int(z / self.step)

        try:
            self.grid[x_index][y_index][z_index] = True
            #self.colors[x_index][y_index][z_index] = color
        except IndexError:
            rospy.logwarn(
                "Point ({}, {}, {}) out of bounds, index ({}, {}, {}) of possible ({}, {}, {})".format(
                    x,
                    y,
                    z,
                    x_index,
                    y_index,
                    z_index,
                    self.x_steps,
                    self.y_steps,
                    self.z_steps,
                )
            )

    def from_points(self, points: List[List[float]], color=(0, 0, 0)) -> None:
        for point in points:
            x, y, z = point
            self.populate_point(x, y, z, color)

    def from_pointcloud2(self, cloud: PointCloud2) -> None:
        rospy.loginfo("Populating voxel grid from pointcloud2")
        arr = cloud_to_array(cloud)
        #colors = rgb_from_pointcloud2(cloud)
        min_x = np.min(arr[:, 0])  # type: ignore
        min_y = np.min(arr[:, 1])  # type: ignore
        min_z = np.min(arr[:, 2])  # type: ignore
        max_x = np.max(arr[:, 0])  # type: ignore
        max_y = np.max(arr[:, 1])  # type: ignore
        max_z = np.max(arr[:, 2])  # type: ignore

        self.resize(max_x - min_x, max_y - min_y, max_z - min_z, self.step)

        # shift the pointcloud so that the minimum point is at the origin
        arr[:, 0] -= min_x
        arr[:, 1] -= min_y
        arr[:, 2] -= min_z
        self.offset = (min_x, min_y, min_z)

        for i in range(len(arr)):
            x, y, z = arr[i]
            self.populate_point(x, y, z)

    def segment(self) -> List["VoxelGrid"]:
        rospy.loginfo("Segmenting voxel grid")
        segments = []
        assert self.grid is not None, "Voxel grid not initialized"
        gridcopy: "VoxelGrid" = VoxelGrid()
        gridcopy.from_voxelgrid(self)
        index = self.find_filled_index(gridcopy)
        while index is not None:
            segment, remainder = self.segment_from_index(
                index, gridcopy, MAX_SEGMENT_SIZE
            )
            segments.append(segment)
            gridcopy = remainder
            index = self.find_filled_index(gridcopy)
        return segments

    def from_voxelgrid(self, voxelgrid: "VoxelGrid") -> None:
        self.resize(voxelgrid.xdim, voxelgrid.ydim, voxelgrid.zdim, voxelgrid.step)
        self.grid = np.copy(voxelgrid.grid)

    def segment_from_index(
        self, index, gridcopy, segment_size_limit
    ) -> Tuple["VoxelGrid", "VoxelGrid"]:
        segment = VoxelGrid(self.xdim, self.ydim, self.zdim, self.step)
        remainder = VoxelGrid(self.xdim, self.ydim, self.zdim, self.step)
        self.segment_from_index_nonrecursive(
            index, gridcopy, segment, remainder, segment_size_limit
        )
        return segment, remainder

    def segment_from_index_nonrecursive(self, index, gridcopy, segment, _, __) -> None:
        segment.populate_point(index[0], index[1], index[2])
        gridcopy.grid[index[0]][index[1]][index[2]] = False
        stack = [index]
        while stack:
            index = stack.pop()
            for neighbor in self.get_occupied_neighbors(index, gridcopy):
                segment.populate_point(neighbor[0], neighbor[1], neighbor[2])
                gridcopy.grid[neighbor[0]][neighbor[1]][neighbor[2]] = False
                stack.append(neighbor)

    def segment_from_index_recursive(
        self,
        index: Tuple[int, int, int],
        gridcopy: "VoxelGrid",
        segment: "VoxelGrid",
        remainder,
        segment_size_limit,
    ):
        """
        Recursively segments a voxel grid into two voxel grids, one containing
        the segment, and the other containing the remainder. We're operating off of the
        memory address here, so we're operating-in-place instead of returning anything.
        """
        x, y, z = index
        segment.grid[x][y][z] = True
        gridcopy.grid[x][y][z] = False
        neighbors = self.get_occupied_neighbors(index, gridcopy)
        for neighbor in neighbors:
            if np.count_nonzero(segment.grid) < segment_size_limit:
                self.segment_from_index_recursive(
                    neighbor, gridcopy, segment, remainder, segment_size_limit
                )

    def get_occupied_neighbors(
        self, index: Tuple[int, int, int], gridcopy: "VoxelGrid"
    ) -> List[Tuple[int, int, int]]:
        neighbors = self.get_neighbors(index, gridcopy)
        occupied_neighbors = []
        for neighbor in neighbors:
            x, y, z = neighbor
            if gridcopy.grid[x][y][z]:
                occupied_neighbors.append(neighbor)
        return occupied_neighbors

    def get_neighbors(
        self, index: Tuple[int, int, int], gridcopy: "VoxelGrid"
    ) -> List[Tuple[int, int, int]]:
        x, y, z = index
        neighbors: List[Tuple[int, int, int]] = []
        for x_offset in [-1, 0, 1]:
            for y_offset in [-1, 0, 1]:
                for z_offset in [-1, 0, 1]:
                    if x_offset == 0 and y_offset == 0 and z_offset == 0:
                        continue
                    try:
                        if gridcopy.grid[x + x_offset][y + y_offset][z + z_offset]:
                            neighbors.append((x + x_offset, y + y_offset, z + z_offset))
                    except IndexError:
                        pass  # this can happen if we're reaching outside the grid... just ignore it, it doesn't matter
        return neighbors

    def get_neighbor(
        self, x: int, y: int, z: int, direction: str, gridcopy: "VoxelGrid"
    ) -> Union[bool, None]:
        if direction == "up":
            return gridcopy.grid[x][y][z + 1] if z + 1 < self.zdim else None
        elif direction == "down":
            return gridcopy.grid[x][y][z - 1] if z - 1 >= 0 else None
        elif direction == "left":
            return gridcopy.grid[x - 1][y][z] if x - 1 >= 0 else None
        elif direction == "right":
            return gridcopy.grid[x + 1][y][z] if x + 1 < self.xdim else None
        elif direction == "back":
            return gridcopy.grid[x][y - 1][z] if y - 1 >= 0 else None
        elif direction == "front":
            return gridcopy.grid[x][y + 1][z] if y + 1 < self.ydim else None
        else:
            return None

    def find_filled_index(
        self, gridcopy: "VoxelGrid"
    ) -> Union[Tuple[int, int, int], None]:
        # TODO: this is a wasteful operation, since we're always working from 0-0-0 to x-y-z
        for x in range(self.x_steps):
            for y in range(self.y_steps):
                for z in range(self.z_steps):
                    if gridcopy.grid[x][y][z]:
                        return x, y, z
        return None
