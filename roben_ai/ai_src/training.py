import os
import json
import torch
from gym_carla import make_carla_env
from stable_baselines3 import PPO
import torch.nn as nn
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
device = "cuda" if torch.cuda.is_available() else "cpu"

checkpoints_dir = "./checkpoints"
params_file = "./best_params.json"
eval_freq = 50_000


# --- Load configuration ---
if os.path.exists(params_file) and os.path.getsize(params_file) > 0:
    with open(params_file, "r") as f:
        config = json.load(f)
else:
    config = {
        "learning_rate": 3e-4,
        "n_steps": 2048,
        "batch_size": 64,
        "n_epochs": 10,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "clip_range": 0.2,
    }

# --- Create a single CARLA environment ---
env = Monitor(make_carla_env(eval=False))

# --- Initialize or load model ---
os.makedirs(checkpoints_dir, exist_ok=True)
policy_kwargs = dict(
    activation_fn=nn.ReLU,
    net_arch=[dict(pi=[32, 32], vf=[32, 32])],
)

model = PPO(
    policy="MultiInputPolicy",
    env=env,
    verbose=1,
    device=device,
    policy_kwargs=policy_kwargs,
    **config,
)

# Optionally resume:
model_path = os.path.join(checkpoints_dir, "best_model.zip")
if os.path.exists(model_path):
    model = PPO.load(model_path, env=env, device=device, policy_kwargs=policy_kwargs, **config)

# --- Callbacks ---
checkpoint_callback = CheckpointCallback(
    save_freq=eval_freq,
    save_path=checkpoints_dir,
    name_prefix="ppo_model",
)

# Evaluate and save best model
eval_callback = EvalCallback(
    env,
    best_model_save_path=checkpoints_dir,
    log_path="./logs",
    eval_freq=eval_freq,
    n_eval_episodes=5,
    deterministic=True,
    render=False,
)

callbacks = [checkpoint_callback, eval_callback]

# --- Training ---
total_timesteps = int(1e7)
model.learn(
    total_timesteps=total_timesteps,
    callback=callbacks,
)

# --- Save final parameters ---
model.save(os.path.join(checkpoints_dir, "final_model.zip"))
with open(params_file, "w") as f:
    json.dump(config, f)
