import torch
import torch.nn as nn
import numpy as np
from config import HyperParams
from utils import Transition
import gymnasium as gym
from torchvision import transforms
from models import Actor, Critic, Backbone
from torch.distributions import Normal


class Trainer:
    def __init__(
        self,
        env: gym.Env,
        opt: torch.optim.Optimizer,
        backbone_net: Backbone,
        actor_net: Actor,
        critic_net: Critic,
        target_net: Critic,
    ):
        self.env = env
        self.backbone_net = backbone_net
        self.actor_net = actor_net
        self.critic_net = critic_net
        self.target_net = target_net
        self.opt = opt

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
            ]
        )

    def start_training(self):
        transitions = []

        obs = self._reset_env()

        for epoch in range(HyperParams.epochs):
            self._rollout(obs)
            self._train()
            self._opt_step()

    def _rollout(self, obs: torch.Tensor):
        for rollout in range(HyperParams.n_rollout):
            throttle_dist, steer_dist = self.actor_net(obs)

            throttle = throttle_dist.sample()
            throttle_log_prob = throttle_dist.log_prob(throttle)
            steer = steer_dist.sample()
            steer_log_prob = steer_dist.log_prob(steer)

            action = [throttle, steer]

            next_obs, reward, terminated, truncated, _ = self.env.step(action)
            print("we need to know how observations look like???")

    def _train(self):
        pass

    def _opt_step(self):
        pass

    def _reset_env(self) -> torch.Tensor:
        obs, _ = self.env.reset()
        obs = self.transform(obs)
        return obs
