from gym_carla import make_carla_env
from stable_baselines3 import PPO
import torch.nn as nn
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
import json
from stable_baselines3.common.monitor import Monitor
import gymnasium as gym
import optuna


def save_best(study, trial):
    # When this trial becomes the new best, write its params to disk
    if study.best_trial == trial:
        with open("best_params.json", "w") as f:
            json.dump(study.best_params, f)


policy_kwargs = dict(
    activation_fn=nn.ReLU,
    net_arch=dict(pi=[32, 32], vf=[32, 32]),
)


def sample_config(trial):
    return {
        # Learning rate: explore orders of magnitude between 1e-5 and 1e-2
        "learning_rate": trial.suggest_float("learning_rate", 1e-5, 1e-2, log=True),
        # Number of steps per env before each update (rollout length)
        "n_steps": trial.suggest_categorical("n_steps", [64, 128, 256, 512]),
        # Minibatch size for each gradient step
        "batch_size": trial.suggest_categorical("batch_size", [8, 16, 32, 64]),
        # Number of passes over the data (epochs) per update
        "n_epochs": trial.suggest_int("n_epochs", 3, 10),
        # Discount factor for future rewards
        "gamma": trial.suggest_uniform("gamma", 0.90, 0.9999),
        # GAE lambda: bias vs variance trade-off
        "gae_lambda": trial.suggest_uniform("gae_lambda", 0.80, 1.00),
        # PPO clipping parameter for the surrogate objective
        "clip_range": trial.suggest_uniform("clip_range", 0.10, 0.30),
        # Optional clipping parameter for the value function
        "clip_range_vf": trial.suggest_categorical("clip_range_vf", [None, 0.10, 0.20, 0.30]),
        # Whether to normalize the advantage estimates
        "normalize_advantage": trial.suggest_categorical("normalize_advantage", [True, False]),
        # Entropy coefficient: encourages exploration
        "ent_coef": trial.suggest_loguniform("ent_coef", 1e-8, 1e-1),
        # Value function loss coefficient
        "vf_coef": trial.suggest_uniform("vf_coef", 0.10, 1.00),
        # Gradient clipping norm
        "max_grad_norm": trial.suggest_uniform("max_grad_norm", 0.30, 1.00),
        # Frequency of SDE noise resampling (only if use_sde=True)
        "sde_sample_freq": trial.suggest_int("sde_sample_freq", -1, 10),
        # Target KL divergence for early stopping (None = no limit)
        "target_kl": trial.suggest_categorical("target_kl", [None, 0.01, 0.05, 0.10]),
    }


def objective(trial):
    train_env = Monitor(make_carla_env())
    config = sample_config(trial)
    # train_env = make_vec_env(make_carla_env, n_envs=2, vec_env_cls=SubprocVecEnv)
    # train_env = make_carla_env()
    # eval_env = Monitor(make_carla_env())

    model = PPO(
        policy="MultiInputPolicy",
        env=train_env,
        verbose=0,
        policy_kwargs=policy_kwargs,
        tensorboard_log="./runs",
        **config,
    )

    eval_interval = 5  # timesteps between evaluations, test 10000 later
    total_timesteps = 100_000

    mean_reward = float("-inf")
    for ts in range(0, total_timesteps, eval_interval):
        model.learn(eval_interval)

        mean_reward, _ = evaluate_policy(model, train_env, n_eval_episodes=5, deterministic=True)

        trial.report(mean_reward, step=ts + eval_interval)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

        print(f"timestep: {ts}, mean_reward: {mean_reward}")

    # train_env._clear_all_actors()
    return mean_reward


if __name__ == "__main__":
    study = optuna.create_study(
        study_name="ppo_carla",  # unique identifier
        storage="sqlite:///ppo_carla.db",  # persistent SQLite DB
        load_if_exists=True,  # resume if DB exists
        direction="maximize",  # we’re maximizing reward
        sampler=optuna.samplers.TPESampler(multivariate=True, group=True),
    )

    # Run 50 trials in parallel if you have CPUs/GPUs available
    study.optimize(objective, n_trials=50, n_jobs=1, callbacks=[save_best])

    # After optimization completes:
    print("Best hyperparameters:", study.best_params)


print("end")
