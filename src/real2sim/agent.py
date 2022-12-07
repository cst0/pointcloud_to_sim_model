from real2sim.validator import ObjectPositionValidator
from real2sim.action_generation.actions import KinovaActions
from real2sim.observation_generation.observers import KinovaObserver
from real2sim.scene_creation.cloudcollector import PointCloud2Collector
import stable_baselines3 as sb3

class Agent:
    """
    Create an agent from stablebaselines which produces a policy to learn
    the current task.
    """

    def __init__(self):
        self.pointcloud_collector = PointCloud2Collector("/camera/depth/color/points")
        self.pointcloud_collector.service_callback(None)
        self.physics_client = self.pointcloud_collector.physics_client
        self.validator = ObjectPositionValidator(0.5, 0.5, 0.5, 0.1) # TODO: get these from somewhere
        self.actions = KinovaActions(self.physics_client)
        self.observer = KinovaObserver(self.physics_client, 0, 1) # TODO: get these from somewhere

    def get_observation(self):
        return self.observer.get_observation()

    def get_reward(self):
        return self.validator.get_reward(self.get_observation())

    def get_done(self):
        return self.validator.get_done(self.get_observation())

    def get_info(self):
        return self.validator.get_info(self.get_observation())

    def step(self, action):
        self.actions.take_action_sim(action)
        return self.get_observation(), self.get_reward(), self.get_done(), self.get_info()

    def step_real(self, action):
        self.actions.take_action_real(action)
        return self.get_observation(), self.get_reward(), self.get_done(), self.get_info()

    def reset(self):
        self.actions.reset()
        return self.get_observation()

    def learn(self):
        model = sb3.HER("MlpPolicy", self, verbose=1)
        model.learn(total_timesteps=1000)
        model.save("ppo_kinova")

    def run(self):
        model = sb3.HER.load("ppo_kinova")
        obs = self.reset()
        done = False
        for _ in range(1000):
            action, _states = model.predict(obs)
            obs, rewards, done, info = self.step(action)

        return done

    def run_on_real(self):
        model = sb3.HER.load("ppo_kinova")
        obs = self.reset()
        done = False
        while not done:
            action, _states = model.predict(obs)
            obs, rewards, done, info = self.step_real(action)

if __name__ == "__main__":
    agent = Agent()
    if agent.learn():
        agent.run_on_real()
    else:
        print("Failed to learn")
