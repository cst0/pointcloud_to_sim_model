import abc
import numpy as np


class Validator(object):
    @abc.abstractmethod
    def get_success(self, observation: np.ndarray) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def get_reward(self, observation: np.ndarray) -> float:
        raise NotImplementedError

class ObjectPositionValidator(Validator):
    def __init__(self, x: float, y: float, z: float, threshold: float):
        self.x = x
        self.y = y
        self.z = z
        self.threshold = threshold

    def get_success(self, observation: np.ndarray) -> bool:
        return np.linalg.norm(observation[:3] - np.array([self.x, self.y, self.z])) < self.threshold

    def get_done(self, observation: np.ndarray) -> bool:
        return self.get_success(observation)

    def get_reward(self, observation: np.ndarray) -> float:
        return 1.0 if self.get_success(observation) else 0.0

    def get_info(self, observation: np.ndarray) -> dict:
        return {"success": self.get_success(observation)}
