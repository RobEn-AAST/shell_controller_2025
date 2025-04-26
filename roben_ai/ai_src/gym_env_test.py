import gym_carla
import gymnasium as gym


if __name__ == "__main__":
    env = gym_carla.make_carla_env()

    obs = env.reset()
    while True:
        action = [2, 0.0]
        obs, r, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        print('epic stuff lol')
        if done:
            obs = env.reset()

# test content
