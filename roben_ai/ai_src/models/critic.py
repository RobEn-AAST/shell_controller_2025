import torch
import torch.nn as nn


class CriticNet(nn.Module):
    def __init__(self, in_size):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(in_size, 32),
            nn.ReLU(),
            nn.Linear(32, 1),
        )

    def forward(self, x):
        y = self.layers(x)
        return y
