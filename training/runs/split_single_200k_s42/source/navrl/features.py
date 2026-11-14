"""Small MLP BEV encoder with a direct route for target and motion scalars."""
import torch
from torch import nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class BevStateExtractor(BaseFeaturesExtractor):
    def __init__(self,observation_space,bev_features=32):
        self.bev_size=observation_space.shape[0]-8
        super().__init__(observation_space,features_dim=bev_features+8)
        self.encoder=nn.Sequential(nn.Linear(self.bev_size,64),nn.ReLU(),nn.Linear(64,bev_features),nn.ReLU())

    def forward(self,observations):
        encoded=self.encoder(observations[:,:self.bev_size])
        return torch.cat([encoded,observations[:,self.bev_size:]],dim=1)
