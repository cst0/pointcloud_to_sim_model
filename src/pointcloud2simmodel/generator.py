#!/usr/bin/env python3
import tempfile
import progressbar

import numpy as np
import rospy
import abc

from pointcloud2simmodel.voxelgrid import VoxelGrid
from pointcloud2simmodel.templates import (
    SDF_HEADER,
    URDF_HEADER,
    SDF_FOOTER,
    URDF_FOOTER,
    SDF_STL_LINK,
    SDF_BOX_JOINT,
    SDF_BOX_LINK,
)


# TODO-- allow setting these via rosparam
# only really used for the box joint
SAFETY_SPACING = 0.01
assert SAFETY_SPACING >= 0


class Generator(object):
    @abc.abstractmethod
    def get_file():
        raise NotImplementedError

    @abc.abstractmethod
    def close_file():
        raise NotImplementedError

class SdfStlGenerator(Generator):
    def __init__(self, voxelgrid: VoxelGrid):
        self.voxelgrid = voxelgrid
        self.openfile_stl = tempfile.NamedTemporaryFile(
            mode="w", suffix=".stl", delete=False
        )
        self.openfile_sdf = tempfile.NamedTemporaryFile(
            mode="w", suffix=".sdf", delete=False
        )

    def write_stl(self) -> None:
        self.openfile_stl.write("solid\n")
        for x in range(self.voxelgrid.x_steps):
            for y in range(self.voxelgrid.y_steps):
                for z in range(self.voxelgrid.z_steps):
                    if self.voxelgrid.grid[x][y][z]:
                        self.write_cube(x, y, z)
        self.openfile_stl.write("endsolid\n")
        self.openfile_stl.flush()

    def write(self) -> None:
        self.write_stl()
        self.openfile_sdf.write(SDF_HEADER.replace("$MODEL_NAME", "voxelgrid"))
        self.openfile_sdf.write(
            SDF_STL_LINK.replace(
                "$NAME", self.openfile_stl.name.split("/")[-1].split(".")[0]
            ).replace("$FILE", self.openfile_stl.name)
        )
        self.openfile_sdf.write(SDF_FOOTER)
        self.openfile_sdf.flush()

    def write_cube(self, x, y, z) -> None:
        self.openfile_stl.write("facet normal 0 0 0\n")
        self.openfile_stl.write("outer loop\n")

        # Define the 8 vertices of the cube
        vertices = np.array(
            [
                [-1, -1, -1],
                [+1, -1, -1],
                [+1, +1, -1],
                [-1, +1, -1],
                [-1, -1, +1],
                [+1, -1, +1],
                [+1, +1, +1],
                [-1, +1, +1],
            ]
        )
        # Define the 12 triangles composing the cube
        faces = np.array(
            [
                [0, 3, 1],  # from -1,-1,-1 to +1,+1,-1 to +1,-1,-1: bottom
                [1, 3, 2],  # from +1,-1,-1 to +1,+1,-1 to -1,+1,-1: bottom
                [0, 4, 7],  # from -1,-1,-1 to -1,-1,+1 to -1,+1,+1: left
                [0, 7, 3],  # from -1,-1,-1 to -1,+1,+1 to -1,+1,-1: left
                [4, 5, 6],  # from -1,-1,+1 to +1,-1,+1 to +1,+1,+1: top
                [4, 6, 7],  # from -1,-1,+1 to +1,+1,+1 to -1,+1,+1: top
                [5, 1, 2],  # from +1,-1,+1 to +1,-1,-1 to +1,+1,-1: right
                [5, 2, 6],  # from +1,-1,+1 to +1,+1,-1 to +1,+1,+1: right
                [2, 3, 6],  # from +1,+1,-1 to -1,+1,-1 to -1,+1,+1: front
                [3, 7, 6],  # from -1,+1,-1 to -1,+1,+1 to +1,+1,+1: front
                [0, 1, 5],  # from -1,-1,-1 to +1,-1,-1 to +1,-1,+1: back
                [0, 5, 4],  # from -1,-1,-1 to +1,-1,+1 to -1,-1,+1: back
            ]
        )

        # mapping of the faces to their names, we'll use this to decide if we should skip a face
        faces_names_map = {
            0: "bottom",
            1: "bottom",
            2: "left",
            3: "left",
            4: "top",
            5: "top",
            6: "right",
            7: "right",
            8: "front",
            9: "front",
            10: "back",
            11: "back",
        }

        # Scale and translate the cube based off of our desired size
        vertices = vertices * self.voxelgrid.step
        vertices[:, 0] += x * self.voxelgrid.step
        vertices[:, 1] += y * self.voxelgrid.step
        vertices[:, 2] += z * self.voxelgrid.step

        # Write the vertices to the file
        for f_index in range(len(faces)):
            face = faces[f_index]
            face_name = faces_names_map[f_index]

            # is there a neighbor in this direction?
            neighbor = self.voxelgrid.get_neighbor(x, y, z, face_name, self.voxelgrid)
            if neighbor:
                # if there is a neighbor, we don't write the face
                continue

            for v_index in face:
                vertex = vertices[v_index]
                self.openfile_stl.write(f"vertex {vertex[0]} {vertex[1]} {vertex[2]}\n")

        self.openfile_stl.write("endloop\n")
        self.openfile_stl.write("endfacet\n")

    def get_file(self):
        return self.openfile_sdf

    def close_file(self):
        self.openfile_stl.close()
        self.openfile_sdf.close()


class UrdfBoxGenerator(SdfStlGenerator):
    def write(self) -> None:
        self.write_stl()
        self.openfile_sdf.write(URDF_HEADER.replace("$MODEL_NAME", "voxelgrid"))
        self.openfile_sdf.write(
            SDF_STL_LINK.replace(
                "$NAME", self.openfile_stl.name.split("/")[-1].split(".")[0]
            ).replace("$FILE", self.openfile_stl.name)
        )
        self.openfile_sdf.write(URDF_FOOTER)
        self.openfile_sdf.flush()



class SdfBoxGenerator(Generator):
    def __init__(self, vg: VoxelGrid, show_progress=False) -> None:
        self.openfile = tempfile.NamedTemporaryFile(
            mode="w", delete=False, suffix=".sdf"
        )
        self.vg = vg
        self.base_link = None
        self.show_progress = show_progress

    def write(self) -> None:
        self.openfile.write(SDF_HEADER)
        self.write_links()
        self.write_joints()
        self.openfile.write(SDF_FOOTER)

    def write_links(self) -> None:
        bar = None
        rospy.loginfo("Writing links")
        if self.show_progress:
            bar = progressbar.ProgressBar(
                maxval=self.vg.x_steps * self.vg.y_steps * self.vg.z_steps
            )
            bar.start()
        for x in range(self.vg.x_steps):
            for y in range(self.vg.y_steps):
                for z in range(self.vg.z_steps):
                    if self.vg.grid[x][y][z]:
                        self.write_link(x, y, z)
                        if self.base_link is None:
                            self.base_link = (x, y, z)
                    if self.show_progress:
                        assert bar is not None
                        bar.update(
                            x * self.vg.y_steps * self.vg.z_steps
                            + y * self.vg.z_steps
                            + z
                        )
        if self.show_progress:
            assert bar is not None
            bar.finish()

    def write_link(self, x: int, y: int, z: int) -> None:
        self.openfile.write(
            SDF_BOX_LINK.replace("$LINKNAME", f"link_{x}_{y}_{z}")
            .replace("$X_POSE", str(round(x * self.vg.step, int(1 / self.vg.step))))
            .replace("$Y_POSE", str(round(y * self.vg.step, int(1 / self.vg.step))))
            .replace("$Z_POSE", str(round(z * self.vg.step, int(1 / self.vg.step))))
            .replace("$ROLL_POSE", "0")
            .replace("$PITCH_POSE", "0")
            .replace("$YAW_POSE", "0")
            .replace("$X_SIZE", str(self.vg.step - SAFETY_SPACING))
            .replace("$Y_SIZE", str(self.vg.step - SAFETY_SPACING))
            .replace("$Z_SIZE", str(self.vg.step - SAFETY_SPACING))
        )

    def write_joints(self) -> None:
        bar = None
        if self.show_progress:
            bar = progressbar.ProgressBar(maxval=self.vg.x_steps * self.vg.y_steps)
            bar.start()

        # write the joints by connecting everything to the base link
        for x in range(self.vg.x_steps):
            for y in range(self.vg.y_steps):
                for z in range(self.vg.z_steps):
                    if self.vg.grid[x][y][z]:
                        self.write_joint(x, y, z)
                    if self.show_progress:
                        assert bar is not None
                        bar.update(x * self.vg.y_steps + y)
        if self.show_progress:
            assert bar is not None
            bar.finish()

    def write_joint(self, x: int, y: int, z: int) -> None:
        if self.base_link is None:
            return
        self.openfile.write(
            SDF_BOX_JOINT.replace("$CHILD", f"link_{x}_{y}_{z}").replace(
                "$PARENT",
                f"link_{self.base_link[0]}_{self.base_link[1]}_{self.base_link[2]}",
            )
        )

    def get_file(self) -> str:
        return self.openfile.name

    def close_file(self) -> None:
        self.openfile.close()

    def __del__(self) -> None:
        self.close_file()
