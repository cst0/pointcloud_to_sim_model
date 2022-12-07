from real2sim.validator import ObjectPositionValidator
from real2sim.action_generation.actions import KinovaActions


class Learner:
    """Create a simple PPO-based agent that learns to achieve the task.
    ObjectPositionValidator provides a reward function, and KinovaActions
    provides the action space."""
