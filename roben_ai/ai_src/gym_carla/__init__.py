
from gymnasium.envs.registration import register
from .envs import CarlaEnv
register(
    id='carla-v0',
    entry_point=CarlaEnv,
)

from .make_env import make_carla_env