# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""
Pinocchio based numerical kinematics of frankik
"""

from __future__ import annotations

import typing

import numpy

__all__: list[str] = ["ClikParameters", "ModelFormat", "PinocchioKinematics", "pinocchio_version"]
M = typing.TypeVar("M", bound=int)

class ClikParameters:
    """
    Parameters of the closed-loop IK iteration.
    """

    max_iterations: int
    def __init__(
        self,
        eps: float = 0.0001,
        max_iterations: int = 1000,
        dt: float = 0.5,
        damping: float = 1e-06,
        clamp_joint_limits: bool = True,
        nullspace_gain: float = 0.0,
    ) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def clamp_joint_limits(self) -> bool:
        """
        Clamp the controlled joints to their limits after every iteration
        """

    @clamp_joint_limits.setter
    def clamp_joint_limits(self, arg0: bool) -> None: ...
    @property
    def damping(self) -> float:
        """
        Levenberg-Marquardt damping
        """

    @damping.setter
    def damping(self, arg0: float) -> None: ...
    @property
    def dt(self) -> float:
        """
        Integration step of the velocity update
        """

    @dt.setter
    def dt(self, arg0: float) -> None: ...
    @property
    def eps(self) -> float:
        """
        Convergence threshold on the SE(3) log error norm
        """

    @eps.setter
    def eps(self, arg0: float) -> None: ...
    @property
    def nullspace_gain(self) -> float:
        """
        Gain pulling the joints towards nullspace_q in the null space of the end-effector task, 0 disables it
        """

    @nullspace_gain.setter
    def nullspace_gain(self, arg0: float) -> None: ...

class ModelFormat:
    """
    Members:

      AUTO : By file extension: .urdf or MJCF

      MJCF

      URDF
    """

    AUTO: typing.ClassVar[ModelFormat]  # value = <ModelFormat.AUTO: 0>
    MJCF: typing.ClassVar[ModelFormat]  # value = <ModelFormat.MJCF: 1>
    URDF: typing.ClassVar[ModelFormat]  # value = <ModelFormat.URDF: 2>
    __members__: typing.ClassVar[
        dict[str, ModelFormat]
    ]  # value = {'AUTO': <ModelFormat.AUTO: 0>, 'MJCF': <ModelFormat.MJCF: 1>, 'URDF': <ModelFormat.URDF: 2>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class PinocchioKinematics:
    """
    Numerical forward/inverse kinematics of a robot described by an MJCF or URDF file. Poses are 4x4 matrices of `tcp_frame` (times an optional tcp offset) relative to `base_frame` (default: world), only the first `dof` configuration variables are controlled.
    """

    parameters: ClikParameters
    def __init__(
        self,
        path: str,
        tcp_frame: str,
        base_frame: str | None = None,
        dof: int | None = None,
        format: ModelFormat = ...,
        parameters: ClikParameters = ...,
    ) -> None: ...
    def forward(
        self,
        q: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]],
        tcp_offset: (
            numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]] | None
        ) = None,
    ) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        """
        Pose of tcp_frame * tcp_offset in base_frame for q (dof or nq entries).
        """

    def frame_names(self) -> list[str]: ...
    def inverse(
        self,
        pose: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]],
        q0: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]],
        tcp_offset: (
            numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]] | None
        ) = None,
        global_solution: bool = False,
    ) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]] | None:
        """
        Controlled joint values reaching pose from seed q0 (dof or nq entries), None if the solver did not converge. global_solution retries from random configurations within the joint limits.
        """

    def joint_names(self) -> list[str]: ...
    def reference_configurations(self) -> dict[str, numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]]:
        """
        MJCF keyframes restricted to the controlled joints
        """

    @property
    def base_frame(self) -> str | None: ...
    @property
    def dof(self) -> int: ...
    @property
    def nq(self) -> int: ...
    @property
    def nullspace_q(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]:
        """
        Posture (dof) the null space task pulls towards
        """

    @nullspace_q.setter
    def nullspace_q(self, arg1: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]) -> None: ...
    @property
    def path(self) -> str: ...
    @property
    def q_max(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]: ...
    @property
    def q_min(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]: ...
    @property
    def q_neutral(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]: ...
    @property
    def q_rest(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]:
        """
        Full configuration (nq) used for the uncontrolled joints
        """

    @q_rest.setter
    def q_rest(self, arg1: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]) -> None: ...
    @property
    def tcp_frame(self) -> str: ...

__version__: str = "1.0.1"
pinocchio_version: str = "3.7.0"
