from gym_carla import make_carla_env
import gymnasium as gym

if __name__ == "__main__":
    env = make_carla_env(eval=True)

    obs = env.reset()
    while True:
        action = [2, 1]
        obs, r, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        print(f'reward: {r}')
        if done:
            obs = env.reset()

# test content
