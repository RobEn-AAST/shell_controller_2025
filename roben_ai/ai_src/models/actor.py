import torch
import torch.nn as nn
from torch.distributions import Normal, Categorical


class Actor(nn.Module):
    def __init__(self, in_size):
        """
        /brake_command [std_msgs/Float64]
            Valid values range from 0.0 (no brake) to 1.0 (full brake)
        gear_command [std_msgs/String]
            Valid values are "forward" or "reverse"
        steering_command [std_msgs/Float64]
            Valid values range from -1.0 (full left) to 1.0 (full right)
        throttle_command [std_msgs/Float64]
            Valid values range from 0.0 (no throttle) to 1.0 (full throttle)
        """
        super().__init__()

        self.body = nn.Sequential(
            nn.Linear(in_size, 32),
            nn.ReLU(),
        )

        # self.gear_head = nn.Sequential(
        #     nn.Linear(32, 2),  # categorical
        # )

        self.steering_head = nn.Sequential(  # -1 - 1
            nn.Linear(32, 16),
            nn.ReLU()(),
            nn.Linear(16, 1),
            nn.Tanh(),
        )
        steering_logstd_param = nn.Parameter(torch.full([1], 0.1))
        self.register_parameter("steering_logstd", steering_logstd_param)

        self.throttle_brake_head = nn.Sequential(  # -1-1
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
            nn.Tanh(),
        )
        throttle_logstd_param = nn.Parameter(torch.full([1], 0.1))
        self.register_parameter("throttle_logstd", throttle_logstd_param)

    def forward(self, x: torch.Tensor) -> tuple[Normal, Normal]:
        """
        x is expected to be the backbone structure
        returns: throttle, steer
        """
        x = self.body(x)

        steering_mue, steering_log_std = self.steering_head(x)
        steering_std = torch.clamp(self.steering_logstd.exp(), 1e-3, 2.0)  # type: ignore
        steering_dist = Normal(steering_mue, steering_std)

        throttle_mue, throttle_log_std = self.throttle_brake_head(x)
        throttle_std = torch.clamp(self.throttle_logstd.exp(), 1e-3, 1.0)  # type: ignore
        throttle_dist = Normal(throttle_mue, throttle_std)

        # gear = self.gear_head(x)
        # gear_dist = Categorical(logits=gear)

        return throttle_dist, steering_dist  # , gear_dist
