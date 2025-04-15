# %%
import torch
import torch.nn as nn
from torchvision import models

# %%


class Backbone(nn.Module):
    output_size = 768

    def __init__(self):
        backbone_base = models.convnext_small(models.ConvNeXt_Small_Weights.DEFAULT)
        self.backbone = nn.Sequential(
            backbone_base.features,
            backbone_base.avgpool,
            backbone_base.classifier[:-1],
        )

    def forward(self, x: torch.Tensor):
        return self.backbone(x)
