import torch
from torch import nn, Tensor
import numpy as np


class PositionalEncoding(nn.Module):
    """Positional Encoding for batched sequences in a form [batch_size, seq_len,
    embedding_dim], encoding is appended to the embedding_dim, i.e. produces
    [batch_size, seq_len, embedding_dim + size]. The values of appended array depends
    on the index of sequence."""

    def __init__(self, size: int, max_len: int = 5000):
        super().__init__()
        position = torch.arange(max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, size, 2) * (-np.log(10000.0) / size))
        pe = torch.zeros(1, max_len, size)
        pe[0, :, 0::2] = torch.sin(position * div_term)
        pe[0, :, 1::2] = torch.cos(position * div_term)
        self.register_buffer("pe", pe)

    def get_encoding(self, x: Tensor) -> Tensor:
        """Get the positional encoding for the given input."""
        return self.pe[:, : x.shape[1], :].expand(x.shape[0], -1, -1)

    def forward(self, x: Tensor) -> Tensor:
        """
        :param x: Tensor, shape [batch_size, seq_len, embedding_dim]
        :return: Tensor of shape [batch_size, seq_len, embedding_dim + size]
        """
        return torch.cat((x, self.get_encoding(x)), dim=-1)
