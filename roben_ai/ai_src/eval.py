from gym_carla import make_carla_env
import gymnasium as gym

if __name__ == "__main__":
    print('making environment...')
    env = make_carla_env(eval=True)
    print('made environment...')


    obs = env.reset()
    print('resetted...')
    while True:
        action = [2, 1]
        obs, r, terminated, truncated, _ = env.step(action)
        print('stepped...')
        done = terminated or truncated
        print(f'reward: {r}')
        if done:
            # break
            obs = env.reset()