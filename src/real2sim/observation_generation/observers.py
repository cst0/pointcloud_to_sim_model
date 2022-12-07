import numpy as np

class KinovaObserver:
    """
    Observe the current state of the robot and the environment.
    """

    def __init__(self, physics_client, object_id, gripper_id):
        self.physics_client = physics_client
        self.object_id = object_id
        self.gripper_id = gripper_id

    def get_observation(self):
        """
        Get the current observation of the robot and the environment.
        """
        # from the pybullet env, get the position of the gripper
        gripper_pos, gripper_orn = self.physics_client.getBasePositionAndOrientation(self.gripper_id)
        # from the pybullet env, get the position of the object
        object_pos, object_orn = self.physics_client.getBasePositionAndOrientation(self.object_id)

        # return the observation
        return np.array([*gripper_pos, *gripper_orn, *object_pos, *object_orn])
