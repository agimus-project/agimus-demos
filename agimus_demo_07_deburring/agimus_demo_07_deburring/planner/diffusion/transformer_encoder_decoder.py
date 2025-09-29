# BSD 3-Clause License
#
# Copyright (C) 2025, Arthur Haffemayer.
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.

import torch.nn as nn
from torch import Tensor

from agimus_demo_07_deburring.planner.diffusion.positional_encoding import (
    PositionalEncoding,
)


class TransformerDiffusionEncoderDecoder(nn.Module):
    def __init__(
        self,
        conditioning_encoder: nn.Module,  # ConditioningEncoder
        position_encoding_size: int = 16,
        configuration_size: int = 2,
        configuration_size_embedding: int = 32,
        encoder_embedding_size: int = 32,
        ff_size: int = 1024,
        num_heads: int | None = None,
        dropout: float = 0.1,
        activation: str = "gelu",
        num_encoder_layers: int = 4,
        num_decoder_layers: int = 4,
    ) -> None:
        super().__init__()

        self.conditioning_encoder = conditioning_encoder

        if num_heads is None:
            num_heads = encoder_embedding_size

        self.configuration_embedding = nn.Linear(
            configuration_size, configuration_size_embedding
        )
        self.output_linear = nn.Linear(
            position_encoding_size + configuration_size_embedding, configuration_size
        )

        # Encoder
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=position_encoding_size + encoder_embedding_size,
            nhead=num_heads,
            dim_feedforward=ff_size,
            dropout=dropout,
            activation=activation,
            batch_first=True,
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer, num_layers=num_encoder_layers
        )

        # Decoder
        decoder_layer = nn.TransformerDecoderLayer(
            d_model=position_encoding_size + configuration_size_embedding,
            nhead=num_heads,
            dim_feedforward=ff_size,
            dropout=dropout,
            activation=activation,
            batch_first=True,
        )
        self.transformer_decoder = nn.TransformerDecoder(
            decoder_layer, num_layers=num_decoder_layers
        )
        self.decoder_position_encoding = PositionalEncoding(position_encoding_size)

    def forward(
        self, cond: dict[str, Tensor], sample: Tensor, noising_time_steps: Tensor
    ) -> Tensor:
        memory = self.conditioning_encoder(cond, noising_time_steps)
        bs, T, dof = sample.shape
        assert dof == 7, f"Expected 7 DOF per config, got {dof}"
        sample_emb = self.configuration_embedding(sample.view(bs * T, dof))
        sample_emb = sample_emb.view(bs, T, -1)
        dec_input = self.decoder_position_encoding(sample_emb)
        output = self.transformer_decoder(dec_input, memory)
        return self.output_linear(output)
