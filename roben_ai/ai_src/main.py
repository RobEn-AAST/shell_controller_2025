from gym_carla import make_carla_env
from stable_baselines3 import PPO
from gymnasium import spaces

env, obs_space = make_carla_env()


if __name__ == '__main__':
    