from training import Trainer
import torch
from gym_carla import make_carla_env
from config import HyperParams
from models import ActorNet, CriticNet, BackboneNet

env = make_carla_env()

backbone_net = BackboneNet()
actor_net = ActorNet(BackboneNet.lastconv_output_channels)
critic_net = CriticNet(BackboneNet.lastconv_output_channels)

target_net = CriticNet(BackboneNet.lastconv_output_channels)
target_net.load_state_dict(critic_net.state_dict())

model_parameters = list(actor_net.parameters()) + list(critic_net.parameters())
opt = torch.optim.Adam(model_parameters, HyperParams.lr)
trainer = Trainer(env, opt, backbone_net, actor_net, critic_net, target_net)

if __name__ == '__main__':
    trainer.start_training()