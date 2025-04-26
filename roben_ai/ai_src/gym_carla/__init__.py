from gymnasium.envs.registration import register
from .make_env import make_carla_env

register(
    id='carla-v0',
    entry_point='gym_carla.envs:CarlaEnv',
)