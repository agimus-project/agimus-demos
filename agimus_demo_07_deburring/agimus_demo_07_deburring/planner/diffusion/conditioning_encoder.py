# BSD 3-Clause License
#
# Copyright (C) 2025, Arthur Haffemayer.
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.

import torch
import torch.nn as nn
from torch import Tensor

from agimus_demo_07_deburring.planner.diffusion.positional_encoding import (
    PositionalEncoding,
)


class ConditioningEncoder(nn.Module):
    def __init__(
        self,
        condition_shapes: dict[str, int],
        position_encoding_size: int,
        encoder_embedding_size: int,
    ) -> None:
        super().__init__()
        self.position_encoding_size = position_encoding_size
        self.encoder_embedding_size = encoder_embedding_size

        self.conditioners = nn.ModuleDict(
            {
                k: nn.Linear(v, encoder_embedding_size)
                for k, v in condition_shapes.items()
            }
        )

        self.num_conditions = len(self.conditioners)

        self.token_type_embedding = nn.Embedding(
            num_embeddings=self.num_conditions + 1,  # +1 for timestep token
            embedding_dim=position_encoding_size,
        )

        self.noising_time_steps_embedding = nn.Sequential(
            nn.Linear(position_encoding_size, position_encoding_size),
            nn.SiLU(),
            nn.Linear(position_encoding_size, encoder_embedding_size),
        )

        self.noising_position_encoding = PositionalEncoding(position_encoding_size)

    def forward(self, cond: dict[str, Tensor], noising_time_steps: Tensor) -> Tensor:
        # squeeze (B,1,7) -> (B,7) if needed
        bs = next(iter(cond.values())).shape[0]

        cond_embs = []
        token_indices = []
        token_type_id = 0

        for key, emb_layer in self.conditioners.items():
            token = cond[key]
            if token.ndim == 3 and token.shape[1] == 1:
                token = token.squeeze(1)  # (B,7)
            token_emb = emb_layer(token).unsqueeze(1)  # (B,1,emb_dim)
            cond_embs.append(token_emb)
            token_indices.append(
                torch.full((bs, 1), token_type_id, device=token.device)
            )
            token_type_id += 1

        cond_tokens = torch.cat(cond_embs, dim=1)  # (B,num_tokens,emb_dim)
        token_indices = torch.cat(token_indices, dim=1)  # (B,num_tokens)
        encoder_pos = self.token_type_embedding(token_indices)  # (B,num_tokens,pos_dim)
        encoder_input = torch.cat(
            [cond_tokens, encoder_pos], dim=-1
        )  # (B,num_tokens,emb+pos)

        # timestep embedding
        t_indices = torch.clamp(
            noising_time_steps, max=self.noising_position_encoding.pe.shape[1] - 1
        )
        t_pe = self.noising_position_encoding.pe[0, t_indices, :]  # (B,pos_dim)
        t_emb = self.noising_time_steps_embedding(t_pe).unsqueeze(1)  # (B,1,emb_dim)
        t_index = torch.full((bs, 1), token_type_id, device=t_emb.device)
        t_pos = self.token_type_embedding(t_index)  # (B,1,pos_dim)
        t_token = torch.cat([t_emb, t_pos], dim=-1)  # (B,1,emb+pos)

        return torch.cat([encoder_input, t_token], dim=1)  # (B,num_tokens+1,emb+pos)
