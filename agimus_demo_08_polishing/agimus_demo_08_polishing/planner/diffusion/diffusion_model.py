# BSD 3-Clause License
#
# Copyright (C) 2025, Arthur Haffemayer.
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.

from agimus_demo_08_polishing.planner.diffusion.conditioning_encoder import (
    ConditioningEncoder,
)
from agimus_demo_08_polishing.planner.diffusion.diffusion_motion import (
    DiffusionMotion,
)
from agimus_demo_08_polishing.planner.diffusion.transformer_encoder_decoder import (
    TransformerDiffusionEncoderDecoder,
)


class DiffusionModel(DiffusionMotion):
    def __init__(self) -> None:
        """
        Model that uses transformer inside diffusion.
        """
        # Conditioning encoder
        conditioning_encoder = ConditioningEncoder(
            condition_shapes={
                "q0": 7,
                "goal": 7,
            },
            encoder_embedding_size=16,
            position_encoding_size=16,
        )

        model = TransformerDiffusionEncoderDecoder(
            configuration_size=7,
            configuration_size_embedding=16,
            encoder_embedding_size=16,
            position_encoding_size=16,
            ff_size=4084,
            dropout=0.01,
            num_decoder_layers=4,
            num_encoder_layers=4,
            num_heads=4,
            conditioning_encoder=conditioning_encoder,
        )
        super().__init__(model, default_diffusion_steps=100)
