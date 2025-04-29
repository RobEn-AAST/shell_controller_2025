# %%
import torch
import torch.nn as nn
from torchvision import models
from functools import partial
from torchvision.models.convnext import LayerNorm2d

# %%

class BackboneNet(nn.Module):
    lastconv_output_channels = 768

    def __init__(self):
        super().__init__()
        
        backbone_base = models.convnext_small(
            weights=models.ConvNeXt_Small_Weights.DEFAULT, out_channels=700
        )
        backbone_base = models.convnext_small(
            weights=models.ConvNeXt_Small_Weights.DEFAULT, out_channels=100
        )

        norm_layer = partial(nn.LayerNorm, eps=1e-6)
        self.backbone = nn.Sequential(
            backbone_base.features,
            backbone_base.avgpool,
            LayerNorm2d((BackboneNet.lastconv_output_channels,), eps=1e-06, elementwise_affine=True),  # type: ignore
            nn.Flatten(start_dim=1, end_dim=1),
        )

    def forward(self, x: torch.Tensor):
        return self.backbone(x)



if __name__ =='__main__':
    backbone = BackboneNet()