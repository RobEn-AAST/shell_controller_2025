
from gymnasium.envs.registration import register
register(
    id='carla-v0',
    entry_point='gym_carla.envs:CarlaEnv',
)

from .make_env import make_carla_env