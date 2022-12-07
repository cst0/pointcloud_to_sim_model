import abc
import os
import rospy

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    rospy.logwarn("pybullet not installed, cannot spawn models with this strategy")

from generator import Generator


class Spawner(object):
    def __init__(self, show_gui=False) -> None:
        self.show_gui = show_gui
        self.client = None
        self.objects = []
        self.start_locations = []

    @abc.abstractmethod
    def spawn(self, generator: Generator):
        raise NotImplementedError


class GazeboSpawner(Spawner):
    def spawn(self, generator: Generator):
        openfile = generator.get_file()
        try:
            cmd = "rosrun gazebo_ros spawn_model -sdf -model {} -file {}".format(
                openfile.name.split("/")[-1].split(".")[0], openfile.name
            )
            rospy.loginfo(f"Running command: {cmd}")
            openfile.flush()
            os.system(cmd)
        except Exception as e:
            rospy.logerr(f"Error spawning model: {e}")
        finally:
            generator.close_file()


class IgnitionSpawner(Spawner):
    def spawn(self, generator: Generator):
        openfile = generator.get_file()
        try:
            cmd = "ign model -m {} -f {}".format(
                openfile.name.split("/")[-1].split(".")[0], openfile.name
            )
            rospy.loginfo(f"Running command: {cmd}")
            openfile.flush()
            os.system(cmd)
        except Exception as e:
            rospy.logerr(f"Error spawning model: {e}")
        finally:
            generator.close_file()


class PybulletSpawner(Spawner):
    def construct_client(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        return self.client

    def spawn(self, generator: Generator):
        if self.client is None:
            self.construct_client()

        openfile = generator.get_file()
        try:
            obj_id = p.loadSDF(openfile.name)
            self.objects.append(obj_id)
        except Exception as e:
            rospy.logerr(f"Error spawning model: {e}")
        finally:
            generator.close_file()
