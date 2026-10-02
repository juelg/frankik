"""frankik: fast analytical inverse kinematics for Franka robots plus a generic numerical solver.

Two solvers are available:

* :class:`FrankaKinematics` -- the blazing fast closed form solver for the Franka Panda and FR3
  (no external dependencies).
* :class:`PinocchioKinematics` -- a numerical damped least squares (CLIK) solver backed by
  `Pinocchio <https://github.com/stack-of-tasks/pinocchio>`_ that works with any robot described by
  a MuJoCo MJCF (or URDF) file. Requires the optional ``pinocchio`` extra (``pip install frankik[pinocchio]``).

Kinematics-only MJCF models of the Panda and FR3 (taken from the MuJoCo Menagerie) are shipped with the
package and can be selected via :class:`RobotType`.
"""

from __future__ import annotations

import importlib
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from importlib import resources
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING, Any

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

# Classic Franka "home" configuration (used as default IK seed), identical to FrankaKinematics.q_home.
Q_HOME_FRANKA = np.array(kQDefault, dtype=np.float64)


@dataclass(frozen=True)
class RobotDescription:
    """Everything needed to load one of the bundled robot models into :class:`PinocchioKinematics`."""

    name: str
    """Name of the bundled MJCF file (without extension) in ``frankik/models``."""
    tcp_frame: str
    """MJCF site (or body/joint) that serves as end-effector frame."""
    base_frame: str | None
    """Body the poses are expressed in (``None`` means the MJCF world frame)."""
    dof: int
    """Number of controlled joints (the first ``dof`` configuration variables of the model)."""
    q_home: np.ndarray
    """Default home / seed configuration."""

    @property
    def mjcf_path(self) -> Path:
        """Absolute path to the bundled MJCF file."""
        return Path(str(resources.files("frankik").joinpath("models", f"{self.name}.xml")))


class RobotType(str, Enum):
    """Robots that ship with frankik.

    The enum values are plain strings (``"panda"``, ``"fr3"``) for backwards compatibility with frankik 1.x,
    i.e. ``RobotType.FR3 == "fr3"``.
    """

    PANDA = "panda"
    FR3 = "fr3"

    @property
    def description(self) -> RobotDescription:
        """Model description (bundled MJCF path, frames, dof, home pose) of this robot."""
        return _ROBOT_DESCRIPTIONS[self]

    @property
    def mjcf_path(self) -> Path:
        """Absolute path to the bundled kinematics-only MuJoCo Menagerie MJCF file."""
        return self.description.mjcf_path


_ROBOT_DESCRIPTIONS: dict[RobotType, RobotDescription] = {
    RobotType.PANDA: RobotDescription(
        name="panda", tcp_frame="attachment_site", base_frame="link0", dof=7, q_home=Q_HOME_FRANKA
    ),
    RobotType.FR3: RobotDescription(
        name="fr3", tcp_frame="attachment_site", base_frame="base", dof=7, q_home=Q_HOME_FRANKA
    ),
}


def pose_inverse(T: np.ndarray) -> np.ndarray:
    """Compute the inverse of a homogeneous transformation matrix.
    Args:
        T (np.ndarray): A 4x4 homogeneous transformation matrix.
    Returns:
        np.ndarray: The inverse of the input transformation matrix.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


class Kinematics(ABC):
    """Common interface of all frankik solvers.

    Poses are 4x4 homogeneous transformation matrices of the tool center point (TCP) relative to the robot
    base. The optional ``tcp_offset`` is a 4x4 transformation from the robot's flange/attachment frame to the
    TCP.
    """

    @property
    @abstractmethod
    def dof(self) -> int:
        """Number of controlled joints."""

    @property
    @abstractmethod
    def q_min(self) -> np.ndarray:
        """Lower joint position limits, shape (dof,)."""

    @property
    @abstractmethod
    def q_max(self) -> np.ndarray:
        """Upper joint position limits, shape (dof,)."""

    @property
    @abstractmethod
    def q_home(self) -> np.ndarray:
        """Default home configuration used as IK seed, shape (dof,)."""

    @abstractmethod
    def forward(self, q0: np.ndarray, tcp_offset: np.ndarray | None = None) -> np.ndarray:
        """Compute the forward kinematics for the given joint configuration."""

    @abstractmethod
    def inverse(
        self,
        pose: np.ndarray,
        q0: np.ndarray | None = None,
        tcp_offset: np.ndarray | None = None,
        **kwargs: Any,
    ) -> np.ndarray | None:
        """Compute the inverse kinematics for the given TCP pose. Returns ``None`` if no solution was found."""

    @staticmethod
    def pose_inverse(T: np.ndarray) -> np.ndarray:
        """Compute the inverse of a homogeneous transformation matrix."""
        return pose_inverse(T)


class FrankaKinematics(Kinematics):
    """Analytical (closed form) kinematics for the Franka Panda and FR3 (He & Liu, 2021)."""

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
        """Initialize Franka Kinematics for the specified robot type.
        Args:
            robot_type (RobotType | str): Type of the robot, either 'panda' or 'fr3'.
        Raises:
            ValueError: If an unsupported robot type is provided.
        """
        try:
            self.robot_type = RobotType(robot_type)
        except ValueError as e:
            msg = f"Unsupported robot type: {robot_type}. Choose 'panda' or 'fr3'."
            raise ValueError(msg) from e
        self._q_min = np.array(q_min_fr3 if self.robot_type == RobotType.FR3 else q_min_panda)
        self._q_max = np.array(q_max_fr3 if self.robot_type == RobotType.FR3 else q_max_panda)
        self._q_home = np.array(kQDefault)

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

    def forward(
        self,
        q0: np.ndarray,
        tcp_offset: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compute the forward kinematics for the given joint configuration.
        Args:
            q0 (np.ndarray): A 7-element array representing joint angles.
            tcp_offset (np.ndarray, optional): A 4x4 homogeneous transformation matrix representing
                the tool center point offset. Defaults to None.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix representing the end-effector pose.
        """
        pose = fk(q0)
        # pose with franka hand tcp offset
        return pose @ self.FrankaHandTCPOffset @ pose_inverse(tcp_offset) if tcp_offset is not None else pose  # type: ignore

    def inverse(
        self,
        pose: np.ndarray,
        q0: np.ndarray | None = None,
        tcp_offset: np.ndarray | None = None,
        q7: float | None = None,
        global_solution: bool = False,
        joint_weight: np.ndarray | None = None,
        q7_sample_interval=40,
        q7_sample_size=60,
        **kwargs: Any,
    ) -> np.ndarray | None:
        """Compute the inverse kinematics for the given end-effector pose.

        Args:
            pose (np.ndarray): A 4x4 homogeneous transformation matrix representing the desired end-effector pose.
            q0 (np.ndarray, optional): A 7-element array representing the current joint angles. Defaults to None.
            tcp_offset (np.ndarray, optional): A 4x4 homogeneous transformation matrix representing
                the tool center point offset. Defaults to None.
            q7 (float, optional): The angle of the seventh joint, used for FR3 robot IK. If None then it will be sampled. Defaults to None.
            global_solution (bool, optional): Whether to consider global ik solutions. Defaults to False.
            joint_weight (np.ndarray, optional): Weights for calculating the distance between the solution and q0. Defaults to None.
            q7_sample_interval (int, optional): The interval for sampling q7. Defaults to 40.
            q7_sample_size (int, optional): The number of samples for q7. Defaults to 60.

        Returns:
            np.ndarray | None: A 7-element array representing the joint angles if a solution is found; otherwise, None.
        """
        if kwargs:
            msg = f"Unknown keyword arguments: {sorted(kwargs)}"
            raise TypeError(msg)
        if joint_weight is None:
            joint_weight = np.ones(7)
        if q0 is None:
            q0 = self.q_home

        new_pose = pose @ pose_inverse(tcp_offset) if tcp_offset is not None else pose
        is_fr3 = self.robot_type == RobotType.FR3

        def get_min(qs):
            qs = [q for q in qs if not np.isnan(q).any()]
            if len(qs) == 0:
                return np.nan
            q_diffs = np.sum(((np.array(qs) - q0) * joint_weight) ** 2, axis=1)
            return qs[np.argmin(q_diffs)]

        if q7 is not None:
            if not global_solution:
                q = ik(new_pose, q0, q7, is_fr3=is_fr3)  # type: ignore
            else:
                qs = ik_full(new_pose, q0, q7, is_fr3=is_fr3)  # type: ignore
                q = get_min(qs)

        else:
            qs = ik_sample_q7(
                new_pose,
                q0,
                is_fr3=is_fr3,
                sample_size=q7_sample_size,
                sample_interval=q7_sample_interval,
                full_ik=global_solution,
            )  # type: ignore
            q = get_min(qs)

        if np.isnan(q).any():
            return None
        return q


_pin_module: ModuleType | None = None


def _load_pin() -> ModuleType:
    """Import the Pinocchio backed extension module lazily with a helpful error message."""
    global _pin_module  # noqa: PLW0603
    if _pin_module is not None:
        return _pin_module
    try:
        # Importing pinocchio first guarantees that its shared libraries are loaded into the process
        # independent of how the wheel was repaired/relocated.
        import pinocchio  # noqa: F401

        pin_module = importlib.import_module("frankik._pin")
    except ImportError as e:
        msg = (
            "The numerical solver requires Pinocchio. Install it with `pip install 'frankik[pinocchio]'` "
            f"(underlying error: {e})"
        )
        raise ImportError(msg) from e
    _pin_module = pin_module
    return pin_module


def has_pinocchio() -> bool:
    """Whether the Pinocchio based numerical solver (:class:`PinocchioKinematics`) is available."""
    try:
        _load_pin()
    except ImportError:
        return False
    return True


class PinocchioKinematics(Kinematics):
    """Numerical kinematics for arbitrary robots described by a MuJoCo MJCF (or URDF) file.

    The inverse kinematics is the damped least squares closed-loop IK (CLIK) of Pinocchio, i.e. the same
    solver that is used in the Robot Control Stack (RCS). It is roughly 20x slower than
    :class:`FrankaKinematics` but works for any serial kinematic chain.

    Example:
        >>> kin = PinocchioKinematics(RobotType.FR3)
        >>> pose = kin.forward(kin.q_home)
        >>> q = kin.inverse(pose, q0=kin.q_home)

        >>> kin = PinocchioKinematics("my_robot.xml", tcp_frame="tcp_site", base_frame="base_link", dof=6)

    Note:
        Pinocchio's MJCF parser (3.7) ignores the ``pos``/``quat`` of the first body below ``<worldbody>`` and
        drops sites of bodies without joints. Mount robots below an intermediate body and attach TCP sites to
        jointed bodies if you rely on these.
    """

    def __init__(
        self,
        model: RobotType | str | os.PathLike[str],
        tcp_frame: str | None = None,
        base_frame: str | None = None,
        dof: int | None = None,
        q_home: np.ndarray | None = None,
        model_format: _pin.ModelFormat | str = "auto",
        parameters: _pin.ClikParameters | None = None,
        **solver_parameters: Any,
    ):
        """Load a robot model.

        Args:
            model (RobotType | str | PathLike): One of the bundled robots (:class:`RobotType` or its string
                value) or the path to a MuJoCo MJCF (``.xml``) or URDF file.
            tcp_frame (str, optional): Name of the end-effector frame in the model (MJCF site, body or joint).
                Required for custom model files, taken from the robot description for bundled robots.
            base_frame (str, optional): Name of the body/frame that poses are expressed in. Defaults to the
                model's world frame for custom files and to the robot base body for bundled robots.
            dof (int, optional): Number of controlled joints, i.e. the first ``dof`` configuration variables of
                the model. Remaining joints (e.g. gripper fingers) are held fixed. Defaults to all joints.
            q_home (np.ndarray, optional): Default seed configuration. Defaults to the MJCF ``home`` keyframe
                if present, otherwise the neutral configuration.
            model_format (ModelFormat | str, optional): ``"auto"`` (by file extension), ``"mjcf"`` or ``"urdf"``.
            parameters (ClikParameters, optional): Solver parameters (eps, max_iterations, dt, damping,
                clamp_joint_limits). Individual parameters can also be given as keyword arguments, e.g.
                ``PinocchioKinematics(RobotType.FR3, eps=1e-6, max_iterations=200)``.
        Raises:
            ImportError: If Pinocchio is not installed.
            ValueError: If the model file, a frame or ``dof`` is invalid.
        """
        pin = _load_pin()
        self.robot_type: RobotType | None = None
        description: RobotDescription | None = None
        if isinstance(model, RobotType):
            self.robot_type = model
        elif isinstance(model, str) and not os.path.exists(model):
            try:
                self.robot_type = RobotType(model)
            except ValueError:
                pass
        if self.robot_type is not None:
            description = self.robot_type.description
            path = description.mjcf_path
            tcp_frame = description.tcp_frame if tcp_frame is None else tcp_frame
            base_frame = description.base_frame if base_frame is None else base_frame
            dof = description.dof if dof is None else dof
        else:
            path = Path(model)
            if not path.exists():
                msg = f"Robot model file does not exist: {path}"
                raise ValueError(msg)
            if tcp_frame is None:
                msg = "tcp_frame must be given for custom robot models (name of an MJCF site, body or joint)"
                raise ValueError(msg)

        if isinstance(model_format, str):
            try:
                fmt = getattr(pin.ModelFormat, model_format.upper())
            except AttributeError as e:
                msg = f"Unknown model format {model_format!r}, choose 'auto', 'mjcf' or 'urdf'"
                raise ValueError(msg) from e
        else:
            fmt = model_format

        params = pin.ClikParameters() if parameters is None else parameters
        for key, value in solver_parameters.items():
            if not hasattr(params, key):
                msg = f"Unknown solver parameter {key!r}, choose from eps, max_iterations, dt, damping, clamp_joint_limits"
                raise TypeError(msg)
            setattr(params, key, value)

        self._impl: _pin.PinocchioKinematics = pin.PinocchioKinematics(
            str(path), tcp_frame, base_frame, dof, fmt, params
        )
        if q_home is not None:
            self._q_home = np.asarray(q_home, dtype=np.float64)
        elif description is not None:
            self._q_home = np.array(description.q_home, dtype=np.float64)
        else:
            self._q_home = self.reference_configurations.get("home", self._impl.q_neutral)

    # --- model information -------------------------------------------------
    @property
    def dof(self) -> int:
        return self._impl.dof

    @property
    def nq(self) -> int:
        """Size of the full model configuration (``>= dof``)."""
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
        """Full (nq) configuration whose tail defines the values of the uncontrolled joints."""
        return self._impl.q_rest

    @q_rest.setter
    def q_rest(self, value: np.ndarray) -> None:
        self._impl.q_rest = np.asarray(value, dtype=np.float64)

    @property
    def path(self) -> Path:
        """Path of the loaded model file."""
        return Path(self._impl.path)

    @property
    def tcp_frame(self) -> str:
        """Name of the end-effector frame."""
        return self._impl.tcp_frame

    @property
    def base_frame(self) -> str | None:
        """Name of the base frame (``None`` for the world frame)."""
        return self._impl.base_frame

    @property
    def joint_names(self) -> list[str]:
        """Names of all joints in configuration order."""
        return self._impl.joint_names()

    @property
    def frame_names(self) -> list[str]:
        """Names of all frames (bodies, joints, sites) that can be used as ``tcp_frame``/``base_frame``."""
        return self._impl.frame_names()

    @property
    def reference_configurations(self) -> dict[str, np.ndarray]:
        """Named configurations defined in the model (MJCF keyframes), restricted to the controlled joints."""
        return self._impl.reference_configurations()

    @property
    def parameters(self) -> _pin.ClikParameters:
        """Solver parameters. Assign a new :class:`ClikParameters` object to change them."""
        return self._impl.parameters

    @parameters.setter
    def parameters(self, value: _pin.ClikParameters) -> None:
        self._impl.parameters = value

    # --- kinematics ----------------------------------------------------------
    def forward(self, q0: np.ndarray, tcp_offset: np.ndarray | None = None) -> np.ndarray:
        """Compute the forward kinematics for the given joint configuration.
        Args:
            q0 (np.ndarray): Joint configuration with ``dof`` (or ``nq``) entries.
            tcp_offset (np.ndarray, optional): A 4x4 homogeneous transformation matrix from ``tcp_frame`` to the
                tool center point. Defaults to None (identity).
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix of the TCP relative to ``base_frame``.
        """
        return self._impl.forward(np.asarray(q0, dtype=np.float64), tcp_offset)

    def inverse(
        self,
        pose: np.ndarray,
        q0: np.ndarray | None = None,
        tcp_offset: np.ndarray | None = None,
        **kwargs: Any,
    ) -> np.ndarray | None:
        """Compute the inverse kinematics for the given TCP pose.
        Args:
            pose (np.ndarray): A 4x4 homogeneous transformation matrix of the desired TCP pose relative to
                ``base_frame``.
            q0 (np.ndarray, optional): Initial guess with ``dof`` (or ``nq``) entries. Defaults to ``q_home``.
            tcp_offset (np.ndarray, optional): A 4x4 homogeneous transformation matrix from ``tcp_frame`` to the
                tool center point. Defaults to None (identity).
        Returns:
            np.ndarray | None: Joint values of the ``dof`` controlled joints or ``None`` if the solver did not
            converge.
        """
        if kwargs:
            msg = f"Unknown keyword arguments: {sorted(kwargs)}"
            raise TypeError(msg)
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
