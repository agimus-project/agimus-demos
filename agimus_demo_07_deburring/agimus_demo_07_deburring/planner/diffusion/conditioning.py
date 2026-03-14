import torch
import torch.nn as nn
from torch import Tensor

from agimus_demo_07_deburring.planner.diffusion.positional_encoding import (
    PositionalEncoding,
)


class ConditioningEncoder(nn.Module):
    """Encodes conditioning information for diffusion model.

    Processes initial configuration, goal, and timestep information into
    a sequence of tokens for the transformer encoder.
    """

    def __init__(
        self,
        condition_shapes: dict[str, int],
        position_encoding_size: int,
        encoder_embedding_size: int,
    ) -> None:
        """Initialize the conditioning encoder.

        Args:
            condition_shapes: Dictionary mapping condition names to their dimensions
            position_encoding_size: Size of positional encodings
            encoder_embedding_size: Size of encoder embeddings
        """
        super().__init__()
        self.position_encoding_size = position_encoding_size
        self.encoder_embedding_size = encoder_embedding_size

        # Linear embeddings for each condition type (goal, q0, etc.)
        self.conditioners = nn.ModuleDict()
        for k, v in condition_shapes.items():
            self.conditioners[k] = nn.Linear(v, encoder_embedding_size)

        self.num_conditions = len(self.conditioners)

        # Token type embedding for different condition types + timestep
        self.token_type_embedding = nn.Embedding(
            num_embeddings=self.num_conditions + 1,  # +1 for timestep token
            embedding_dim=position_encoding_size,
        )

        # Timestep embedding network
        self.noising_time_steps_embedding = nn.Sequential(
            nn.Linear(position_encoding_size, position_encoding_size),
            nn.SiLU(),
            nn.Linear(position_encoding_size, encoder_embedding_size),
        )

        self.noising_position_encoding = PositionalEncoding(position_encoding_size)

    def forward(self, cond: dict[str, Tensor], noising_time_steps: Tensor) -> Tensor:
        """Forward pass to encode conditioning information.

        Args:
            cond: Dictionary of conditioning tensors, each of shape (B, D_i)
            noising_time_steps: Current timestep in diffusion process, shape (B,) or scalar

        Returns:
            Encoded conditioning tokens of shape (B, T_total, D + pos_dim)
            where T_total = num_conditions + 1 (for timestep)
        """
        # Ensure noising_time_steps is 1D batch tensor
        if noising_time_steps.dim() == 0:
            noising_time_steps = noising_time_steps.unsqueeze(0)
        if noising_time_steps.dim() != 1:
            raise ValueError(
                f"Invalid shape for noising_time_steps: {noising_time_steps.shape}"
            )

        batch_size = noising_time_steps.shape[0]

        # Get device from noising_time_steps
        device = noising_time_steps.device

        cond_embs = []
        token_type_id = 0

        # Process each condition (goal, q0, etc.)
        for key, emb_layer in self.conditioners.items():
            token = cond[key]  # (B, D_i)

            # Verify batch size matches
            if token.shape[0] != batch_size:
                raise ValueError(
                    f"Batch size mismatch: condition '{key}' has batch size "
                    f"{token.shape[0]} but expected {batch_size}"
                )

            # Embed and add sequence dimension
            token_emb = emb_layer(token).unsqueeze(1)  # (B, 1, encoder_embedding_size)

            # Get token type embedding
            token_type_ids = torch.full(
                (batch_size, 1), token_type_id, dtype=torch.long, device=device
            )
            pos_emb = self.token_type_embedding(
                token_type_ids
            )  # (B, 1, pos_encoding_size)

            # Concatenate embedding and positional encoding
            token_with_pos = torch.cat(
                [token_emb, pos_emb], dim=-1
            )  # (B, 1, D+pos_dim)
            cond_embs.append(token_with_pos)

            token_type_id += 1

        # Combine all condition tokens
        encoder_input = torch.cat(cond_embs, dim=1)  # (B, T_cond, D+pos_dim)

        # Process timestep token
        # Clamp indices to valid range
        t_indices = torch.clamp(
            noising_time_steps.long(),
            max=self.noising_position_encoding.pe.shape[1] - 1,
        )

        # Get positional encoding for timesteps
        t_pe = self.noising_position_encoding.pe[0, t_indices, :]  # (B, pos_dim)

        # Embed timestep
        t_emb = self.noising_time_steps_embedding(t_pe).unsqueeze(1)  # (B, 1, D)

        # Add token type embedding for timestep
        t_type = torch.full(
            (batch_size, 1), token_type_id, dtype=torch.long, device=device
        )
        t_pos = self.token_type_embedding(t_type)  # (B, 1, pos_dim)

        # Combine timestep embedding and position
        t_token = torch.cat([t_emb, t_pos], dim=-1)  # (B, 1, D + pos_dim)

        # Concatenate all tokens (conditions + timestep)
        return torch.cat([encoder_input, t_token], dim=1)  # (B, T_total, D + pos_dim)
