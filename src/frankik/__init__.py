"""Fast analytical inverse kinematics for Franka robots and a Pinocchio based numerical solver for any MJCF robot."""

from __future__ import annotations

import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from functools import cache
from importlib import import_module, resources
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING

import numpy as np

from frankik._core import (
    __version__,
    fk,
    ik,
    ik_full,
    ik_sample_q7,
    kQDefault,
    q_max_fr3,
    q_max_panda,
    q_min_fr3,
    q_min_panda,
)

if TYPE_CHECKING:
    import frankik._pin as _pin

Q_HOME_FRANKA = np.asarray(kQDefault, dtype=np.float64)


@dataclass(frozen=True)
class RobotDescription:
    """Bundled MJCF model and the frames needed to load it into :class:`PinocchioKinematics`."""

    name: str
    tcp_frame: str
    base_frame: str
    dof: int
    q_home: np.ndarray

    @property
    def mjcf_path(self) -> Path:
        return Path(str(resources.files("frankik").joinpath("models", f"{self.name}.xml")))


class RobotType(str, Enum):
    """Robots shipped with frankik. Values equal the frankik 1.x string constants ("panda", "fr3")."""

    PANDA = "panda"
    FR3 = "fr3"

    @property
    def description(self) -> RobotDescription:
        return _ROBOT_DESCRIPTIONS[self]

    @property
    def mjcf_path(self) -> Path:
        return self.description.mjcf_path


_ROBOT_DESCRIPTIONS = {
    RobotType.PANDA: RobotDescription("panda", "attachment_site", "link0", 7, Q_HOME_FRANKA),
    RobotType.FR3: RobotDescription("fr3", "attachment_site", "base", 7, Q_HOME_FRANKA),
}


def pose_inverse(T: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 homogeneous transformation matrix."""
    R = T[:3, :3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ T[:3, 3]
    return T_inv


class Kinematics(ABC):
    """Common solver interface. Poses are 4x4 matrices of the TCP relative to the robot base,
    ``tcp_offset`` transforms from the flange/attachment frame to the TCP."""

    pose_inverse = staticmethod(pose_inverse)

    @property
    @abstractmethod
    def dof(self) -> int: ...

    @property
    @abstractmethod
    def q_min(self) -> np.ndarray: ...

    @property
    @abstractmethod
    def q_max(self) -> np.ndarray: ...

    @property
    @abstractmethod
    def q_home(self) -> np.ndarray: ...

    @abstractmethod
    def forward(self, q0: np.ndarray, tcp_offset: np.ndarray | None = None) -> np.ndarray: ...

    @abstractmethod
    def inverse(
        self, pose: np.ndarray, q0: np.ndarray | None = None, tcp_offset: np.ndarray | None = None
    ) -> np.ndarray | None:
        """Joint configuration reaching ``pose`` or ``None`` if no solution was found."""


class FrankaKinematics(Kinematics):
    """Analytical kinematics for the Franka Panda and FR3 (He & Liu, 2021)."""

    FrankaHandTCPOffset = np.array(
        [
            [0.707, 0.707, 0.0, 0.0],
            [-0.707, 0.707, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.1034],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    def __init__(self, robot_type: RobotType | str = RobotType.FR3):
        try:
            self.robot_type = RobotType(robot_type)
        except ValueError as e:
            msg = f"Unsupported robot type: {robot_type}. Choose 'panda' or 'fr3'."
            raise ValueError(msg) from e
        is_fr3 = self.robot_type == RobotType.FR3
        self._q_min = np.array(q_min_fr3 if is_fr3 else q_min_panda)
        self._q_max = np.array(q_max_fr3 if is_fr3 else q_max_panda)
        self.q_home = Q_HOME_FRANKA

    @property
    def dof(self) -> int:
        return 7

    @property
    def q_min(self) -> np.ndarray:
        return self._q_min

    @property
    def q_max(self) -> np.ndarray:
        return self._q_max

    @property
    def q_home(self) -> np.ndarray:
        return self._q_home

    @q_home.setter
    def q_home(self, value: np.ndarray) -> None:
        self._q_home = np.asarray(value, dtype=np.float64)

    def forward(self, q0: np.ndarray, tcp_offset: np.ndarray | None = None) -> np.ndarray:
        """Forward kinematics for ``q0``, see :attr:`FrankaHandTCPOffset` for the TCP convention."""
        pose = fk(q0)
        if tcp_offset is None:
            return pose  # type: ignore
        return pose @ self.FrankaHandTCPOffset @ pose_inverse(tcp_offset)  # type: ignore

    def inverse(
        self,
        pose: np.ndarray,
        q0: np.ndarray | None = None,
        tcp_offset: np.ndarray | None = None,
        q7: float | None = None,
        global_solution: bool = False,
        joint_weight: np.ndarray | None = None,
        q7_sample_interval: int = 40,
        q7_sample_size: int = 60,
    ) -> np.ndarray | None:
        """Solve the IK for ``pose``.

        Args:
            q0: Seed configuration, defaults to ``q_home``. The solution closest to it is returned.
            q7: Fixed angle of the redundant joint 7. Sampled with ``q7_sample_size``/``q7_sample_interval`` (deg) if None.
            global_solution: Consider all elbow/shoulder configurations instead of the one closest to ``q0``.
            joint_weight: Per joint weights for the distance to ``q0``.
        """
        if joint_weight is None:
            joint_weight = np.ones(7)
        if q0 is None:
            q0 = self.q_home
        if tcp_offset is not None:
            pose = pose @ pose_inverse(tcp_offset)
        is_fr3 = self.robot_type == RobotType.FR3

        def closest(qs):
            qs = [q for q in qs if not np.isnan(q).any()]
            if len(qs) == 0:
                return np.nan
            q_diffs = np.sum(((np.array(qs) - q0) * joint_weight) ** 2, axis=1)
            return qs[np.argmin(q_diffs)]

        if q7 is None:
            qs = ik_sample_q7(pose, q0, is_fr3, q7_sample_size, q7_sample_interval, global_solution)  # type: ignore
            q = closest(qs)
        elif global_solution:
            q = closest(ik_full(pose, q0, q7, is_fr3))  # type: ignore
        else:
            q = ik(pose, q0, q7, is_fr3)  # type: ignore
        return None if np.isnan(q).any() else q


@cache
def _load_pin() -> ModuleType:
    try:
        import pinocchio  # noqa: F401  # loads the shared libraries _pin links against

        return import_module("frankik._pin")
    except ImportError as e:
        msg = f"The numerical solver requires Pinocchio, install it with `pip install 'frankik[pinocchio]'` ({e})"
        raise ImportError(msg) from e


def has_pinocchio() -> bool:
    """Whether :class:`PinocchioKinematics` is available."""
    try:
        _load_pin()
    except ImportError:
        return False
    return True


class PinocchioKinematics(Kinematics):
    """Numerical kinematics for any robot described by a MuJoCo MJCF (or URDF) file.

    The IK is the damped least squares closed-loop IK (CLIK) of Pinocchio as used in the Robot Control Stack.
    Poses are expressed relative to ``base_frame`` (the model's world frame if None), only the first ``dof``
    configuration variables are controlled, remaining joints are held at ``q_rest``.

    Pinocchio's MJCF parser ignores the pose of the first body below ``<worldbody>`` and drops sites of bodies
    without joints.

    Example:
        >>> kin = PinocchioKinematics(RobotType.FR3)
        >>> kin = PinocchioKinematics("robot.xml", tcp_frame="tcp_site", base_frame="base_link", dof=6, eps=1e-5)
    """

    def __init__(
        self,
        model: RobotType | str | os.PathLike[str],
        tcp_frame: str | None = None,
        base_frame: str | None = None,
        dof: int | None = None,
        q_home: np.ndarray | None = None,
        model_format: str = "auto",
        parameters: _pin.ClikParameters | None = None,
        **solver_parameters: float | int | bool,
    ):
        """Load a robot model.

        Args:
            model: Bundled robot (:class:`RobotType` or its value) or path to an MJCF/URDF file.
            tcp_frame: End-effector frame (MJCF site, body or joint). Required for custom models.
            base_frame: Frame the poses are expressed in. Defaults to the world frame for custom models.
            dof: Number of controlled joints. Defaults to all joints of the model.
            q_home: Default IK seed. Defaults to the MJCF ``home`` keyframe or the neutral configuration.
            model_format: ``"auto"`` (by file extension), ``"mjcf"`` or ``"urdf"``.
            parameters: Solver parameters, alternatively given as keyword arguments
                (``eps``, ``max_iterations``, ``dt``, ``damping``, ``clamp_joint_limits``, ``restarts``).
        """
        pin = _load_pin()
        self.robot_type = self._as_robot_type(model)
        if self.robot_type is not None:
            description = self.robot_type.description
            path = description.mjcf_path
            tcp_frame = tcp_frame or description.tcp_frame
            base_frame = base_frame or description.base_frame
            dof = dof or description.dof
        else:
            path = Path(model)  # type: ignore[arg-type]
            if not path.exists():
                msg = f"Robot model file does not exist: {path}"
                raise ValueError(msg)
            if tcp_frame is None:
                msg = "tcp_frame must be given for custom robot models"
                raise ValueError(msg)
        try:
            fmt = pin.ModelFormat.__members__[model_format.upper()]
        except KeyError as e:
            msg = f"Unknown model format {model_format!r}, choose 'auto', 'mjcf' or 'urdf'"
            raise ValueError(msg) from e
        if parameters is not None and solver_parameters:
            msg = "Give either `parameters` or individual solver parameters"
            raise TypeError(msg)
        parameters = parameters or pin.ClikParameters(**solver_parameters)

        self._impl: _pin.PinocchioKinematics = pin.PinocchioKinematics(
            str(path), tcp_frame, base_frame, dof, fmt, parameters
        )
        if q_home is not None:
            self.q_home = q_home
        elif self.robot_type is not None:
            self.q_home = self.robot_type.description.q_home
        else:
            self.q_home = self.reference_configurations.get("home", self._impl.q_neutral)

    @staticmethod
    def _as_robot_type(model: RobotType | str | os.PathLike[str]) -> RobotType | None:
        if isinstance(model, RobotType):
            return model
        if isinstance(model, str) and not os.path.exists(model) and model in RobotType._value2member_map_:
            return RobotType(model)
        return None

    @property
    def dof(self) -> int:
        return self._impl.dof

    @property
    def nq(self) -> int:
        """Size of the full model configuration."""
        return self._impl.nq

    @property
    def q_min(self) -> np.ndarray:
        return self._impl.q_min

    @property
    def q_max(self) -> np.ndarray:
        return self._impl.q_max

    @property
    def q_home(self) -> np.ndarray:
        return self._q_home

    @q_home.setter
    def q_home(self, value: np.ndarray) -> None:
        self._q_home = np.asarray(value, dtype=np.float64)

    @property
    def q_rest(self) -> np.ndarray:
        """Full configuration (``nq``) whose tail is used for the uncontrolled joints."""
        return self._impl.q_rest

    @q_rest.setter
    def q_rest(self, value: np.ndarray) -> None:
        self._impl.q_rest = np.asarray(value, dtype=np.float64)

    @property
    def path(self) -> Path:
        return Path(self._impl.path)

    @property
    def tcp_frame(self) -> str:
        return self._impl.tcp_frame

    @property
    def base_frame(self) -> str | None:
        return self._impl.base_frame

    @property
    def joint_names(self) -> list[str]:
        return self._impl.joint_names()

    @property
    def frame_names(self) -> list[str]:
        """Frames (bodies, joints, sites) usable as ``tcp_frame``/``base_frame``."""
        return self._impl.frame_names()

    @property
    def reference_configurations(self) -> dict[str, np.ndarray]:
        """MJCF keyframes restricted to the controlled joints."""
        return self._impl.reference_configurations()

    @property
    def parameters(self) -> _pin.ClikParameters:
        return self._impl.parameters

    @parameters.setter
    def parameters(self, value: _pin.ClikParameters) -> None:
        self._impl.parameters = value

    def forward(self, q0: np.ndarray, tcp_offset: np.ndarray | None = None) -> np.ndarray:
        """Pose of ``tcp_frame`` times ``tcp_offset`` for ``q0`` (``dof`` or ``nq`` entries)."""
        return self._impl.forward(np.asarray(q0, dtype=np.float64), tcp_offset)

    def inverse(
        self, pose: np.ndarray, q0: np.ndarray | None = None, tcp_offset: np.ndarray | None = None
    ) -> np.ndarray | None:
        """Joint values of the controlled joints reaching ``pose`` from seed ``q0`` (default ``q_home``),
        ``None`` if the solver did not converge."""
        if q0 is None:
            q0 = self.q_home
        return self._impl.inverse(np.asarray(pose, dtype=np.float64), np.asarray(q0, dtype=np.float64), tcp_offset)

    def __repr__(self) -> str:
        return (
            f"PinocchioKinematics(model={self.path.name!r}, tcp_frame={self.tcp_frame!r}, "
            f"base_frame={self.base_frame!r}, dof={self.dof})"
        )


__all__ = [
    "__version__",
    "ik",
    "ik_full",
    "ik_sample_q7",
    "fk",
    "kQDefault",
    "q_max_fr3",
    "q_max_panda",
    "q_min_fr3",
    "q_min_panda",
    "Q_HOME_FRANKA",
    "pose_inverse",
    "Kinematics",
    "FrankaKinematics",
    "PinocchioKinematics",
    "RobotDescription",
    "RobotType",
    "has_pinocchio",
]
