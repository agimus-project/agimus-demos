from abc import ABC, abstractmethod

import numpy.typing as npt
import pinocchio as pin


class GenericTrajectoryGenerator(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def get_path(
        self, q0: npt.ArrayLike, T_final: pin.SE3, handle_name: str | None = None
    ) -> list[npt.ArrayLike]:
        pass


class JointSpaceMotionGenerator(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def update_polished_object_pose(self, T_polished_object: pin.SE3) -> None:
        pass
