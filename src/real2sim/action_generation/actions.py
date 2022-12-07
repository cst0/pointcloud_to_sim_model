import abc


class ActionWrapper(object):
    def __init__(self, env):
        self.env = env

    @abc.abstractmethod
    def get_action_list(self) -> list:
        """
        There exists a set of actions that can be performed in the environment.
        This method returns a list of all possible actions.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def take_action_real(self, action: int) -> int:
        """given an action, what's the real-world equivalent?"""
        raise NotImplementedError

    @abc.abstractmethod
    def take_action_sim(self, action: int) -> int:
        """given an action, what's the simulator equivalent?"""
        raise NotImplementedError


class KinovaActions(ActionWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.actions = [
            0,  # do nothing
            1,  # +x
            2,  # -x
            3,  # +y
            4,  # -y
            5,  # +z
            6,  # -z
            7,  # +roll
            8,  # -roll
            9,  # +pitch
            10,  # -pitch
            11,  # +yaw
            12,  # -yaw
            13,  # +gripper
            14,  # -gripper
        ]

        self.sim_action_map = {
            0: 0, # TODO
        }

        self.real_action_map = {
            0: 0, # TODO
        }

    def get_action_list(self) -> list:
        return self.actions

    def take_action_real(self, action: int) -> int:
        return self.real_action_map[action]

    def take_action_sim(self, action: int) -> int:
        return self.sim_action_map[action]

    def reset(self):
        # env is a pybullet client
        # let's move the robot arm back to the starting position of (0, 0, 0)
        self.env.resetBasePositionAndOrientation(self.env.gripper_id, [0, 0, 0], [0, 0, 0, 1])
        # let's move the object back to the starting position of (0, 0, 0)
        self.env.resetBasePositionAndOrientation(self.env.object_id, [0, 0, 0], [0, 0, 0, 1])
