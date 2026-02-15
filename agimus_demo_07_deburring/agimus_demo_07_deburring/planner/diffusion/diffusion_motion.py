from __future__ import annotations

import pytorch_lightning as pl
import torch
import tqdm
from diffusers import DDPMScheduler
from torch import Tensor


class DiffusionMotion(pl.LightningModule):
    """Diffusion model for motion generation with proper batch handling."""

    def __init__(
        self, model=None, noise_scheduler=None, default_diffusion_steps=1000
    ) -> None:
        super().__init__()
        self.model = model

        if noise_scheduler is None:
            self.noise_scheduler = DDPMScheduler(
                num_train_timesteps=default_diffusion_steps,
                beta_schedule="linear",
                clip_sample=False,
            )
        else:
            self.noise_scheduler = noise_scheduler

        self.diffusion_noising_steps = len(self.noise_scheduler.timesteps)

    def _replicate_conditioning(
        self, cond: dict[str, Tensor], target_batch_size: int
    ) -> dict[str, Tensor]:
        """Replicate conditioning tensors to match target batch size.

        Args:
            cond: Dictionary of conditioning tensors
            target_batch_size: Desired batch size

        Returns:
            Dictionary with replicated conditioning tensors
        """
        replicated_cond = {}

        for key, value in cond.items():
            current_batch_size = value.shape[0]

            if current_batch_size == target_batch_size:
                # Already correct batch size
                replicated_cond[key] = value
            elif current_batch_size == 1:
                # Replicate single conditioning to match batch
                replicated_cond[key] = value.repeat(
                    target_batch_size, *([1] * (value.dim() - 1))
                )
            else:
                raise ValueError(
                    f"Conditioning '{key}' has batch size {current_batch_size}, "
                    f"but expected either 1 or {target_batch_size}. "
                    f"Cannot automatically handle this case."
                )

        return replicated_cond

    def single_denoise_step(
        self, xt: Tensor, t: Tensor | int, cond: dict[str, Tensor]
    ) -> Tensor:
        """Perform a single de-noising step on a batch of motions.

        Runs model to get x0 and noise it back by diffusion to get x_{t-1}.
        If t-1 is zero, no diffusion is applied.

        Args:
            xt: Batch of motions (bs, seq_length, config_size)
            t: Current noising timestep of the motion (bs,) or int if same timestep
               should be used for all batches
            cond: Conditioning dictionary with tensors of shape (bs, ...)

        Returns:
            Batch of motions at noising timestep t-1
        """
        if isinstance(t, int):
            t = t * torch.ones((xt.shape[0],), dtype=torch.int, device=self.device)

        # Ensure conditioning batch size matches xt batch size
        batch_size = xt.shape[0]
        cond = self._replicate_conditioning(cond, batch_size)

        # Predict x0 from xt
        x0 = self.model(cond=cond, sample=xt, noising_time_steps=t)

        # Add noise back to get x_{t-1}
        xt_minus_1 = self.generate_x_t(x0, torch.clip(t - 1, min=0))

        # For t-1 <= 0, no diffusion, just use x0
        mask = t <= 1
        xt_minus_1[mask] = x0[mask]

        return xt_minus_1

    def sample(
        self,
        cond: dict[str, Tensor],
        bs: int = 1,
        seq_length: int = 32,
        configuration_size: int = 2,
        projection_fn=None,
        diffusion_steps: int = None,
    ) -> Tensor:
        """Sample a new motion randomly by de-noising gaussian noise.

        Args:
            cond: Conditioning dictionary. Tensors can have batch size 1 or bs.
                  If batch size is 1, they will be automatically replicated to bs.
            bs: Number of samples to create
            seq_length: Length of the sequence
            configuration_size: Size of the configuration
            projection_fn: Projection function that is used to map xt after each
                de-noising step; it should be in form x_projected = f(x, t), t has
                values from T-1 to 0, where T is self.diffusion_noising_steps.
            diffusion_steps: Number of diffusion steps to use. If None, uses
                self.diffusion_noising_steps

        Returns:
            Sampled motions (bs, seq_length, configuration_size)
        """
        # Replicate conditioning if needed
        cond = self._replicate_conditioning(cond, bs)

        with torch.no_grad():
            # Start from random noise
            xt = torch.randn((bs, seq_length, configuration_size), device=self.device)

            # Determine number of steps
            steps = diffusion_steps or self.diffusion_noising_steps

            # Iteratively denoise
            for t in tqdm.trange(steps - 1, 0, -1, desc="Sampling"):
                xt = self.single_denoise_step(xt=xt, t=t, cond=cond)

                if projection_fn is not None:
                    xt = projection_fn(xt, t)

        return xt

    def generate_x_t(self, x0: Tensor, t: Tensor) -> Tensor:
        """Generate x_t from given x_0 and noising time steps t.

        Applies forward process of the diffusion.

        Args:
            x0: Clean samples (bs, seq_len, conf_len)
            t: Noising time steps (bs,)

        Returns:
            Noised samples x_t (bs, seq_len, conf_len)
        """
        assert t.shape == x0.shape[:1], (
            f"Time steps shape {t.shape} must match batch size {x0.shape[0]}"
        )
        return self.noise_scheduler.add_noise(x0, torch.randn_like(x0), t)

    def configure_optimizers(self):
        """Configure optimizer for training."""
        return torch.optim.Adam(self.model.parameters())

    def training_step(self, train_batch, batch_idx):
        """Single training step.

        Args:
            train_batch: Tuple of (samples, conds)
            batch_idx: Batch index

        Returns:
            Training loss
        """
        samples, conds = train_batch
        bs = samples.shape[0]

        # Sample random timesteps
        t = torch.randint(0, self.diffusion_noising_steps, (bs,), device=self.device)

        # Predict x0 from noised samples
        pred = self.model(
            sample=self.generate_x_t(samples, t), noising_time_steps=t, cond=conds
        )

        # MSE loss
        loss = (pred - samples).square().mean()
        self.log("loss", loss)

        return loss
