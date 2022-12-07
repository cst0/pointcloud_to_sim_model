import abc
import os
import rospy

from generator import Generator

class Spawner(object):
    @abc.abstractmethod
    def spawn(self, generator:Generator):
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
    def spawn(self, generator: Generator):
        openfile = generator.get_file()
        try:
            import pybullet as p
            import pybullet_data

            p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.8)
            p.loadURDF("plane.urdf")
            p.loadSDF(openfile.name)

        except ImportError as e:
            rospy.logerr(f"Couldn't import pybullet: {e}")
        except Exception as e:
            rospy.logerr(f"Error spawning model: {e}")
        finally:
            generator.close_file()
