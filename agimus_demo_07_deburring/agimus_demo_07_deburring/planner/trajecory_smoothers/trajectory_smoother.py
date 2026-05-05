from abc import ABC, abstractmethod

import pinocchio as pin
import numpy.typing as npt


class GenericTrajectorySmoother(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def __call__(
        self, q0: npt.ArrayLike, T_final: pin.SE3
    ) -> tuple[npt.ArrayLike, npt.ArrayLike, float] | tuple[None, None, None]:
        pass

    @property
    @abstractmethod
    def n_samples(self) -> int:
        pass

    @abstractmethod
    def update_poses(self, T_pylone: pin.SE3) -> None:
        pass
