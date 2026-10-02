# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""
Python bindings for the Pinocchio based numerical kinematics of frankik
"""

from __future__ import annotations

import typing

import numpy

__all__: list[str] = ["ClikParameters", "ModelFormat", "PinocchioKinematics", "pinocchio_version"]
M = typing.TypeVar("M", bound=int)

class ClikParameters:
    """
    Tuning parameters of the closed-loop inverse kinematics iteration.
    """

    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(
        self,
        eps: float = 0.0001,
        max_iterations: int = 1000,
        dt: float = 0.1,
        damping: float = 1e-06,
        clamp_joint_limits: bool = False,
    ) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def clamp_joint_limits(self) -> bool:
        """
        Clamp controlled joints to their limits after every iteration.
        """

    @clamp_joint_limits.setter
    def clamp_joint_limits(self, arg0: bool) -> None: ...
    @property
    def damping(self) -> float:
        """
        Levenberg-Marquardt damping.
        """

    @damping.setter
    def damping(self, arg0: float) -> None: ...
    @property
    def dt(self) -> float:
        """
        Integration step of the velocity update.
        """

    @dt.setter
    def dt(self, arg0: float) -> None: ...
    @property
    def eps(self) -> float:
        """
        Convergence threshold on the SE(3) log error norm.
        """

    @eps.setter
    def eps(self, arg0: float) -> None: ...
    @property
    def max_iterations(self) -> int:
        """
        Maximum number of iterations.
        """

    @max_iterations.setter
    def max_iterations(self, arg0: int) -> None: ...

class ModelFormat:
    """
    Robot description file format.

    Members:

      AUTO : Guess from the file extension (`.urdf` -> URDF, otherwise MJCF)

      MJCF : MuJoCo XML

      URDF : URDF
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
    Numerical forward/inverse kinematics for arbitrary robots described by a MuJoCo MJCF (or URDF) file, computed with Pinocchio.

    Poses are 4x4 homogeneous matrices of the `tcp_frame` (optionally times a tcp offset) expressed in `base_frame` (default: world).
    """

    def __init__(
        self,
        path: str,
        tcp_frame: str,
        base_frame: str | None = None,
        dof: int | None = None,
        format: ModelFormat = ...,
        parameters: ClikParameters = ...,
    ) -> None:
        """
        Load a robot model.

        Args:
            path (str): Path to the MJCF (or URDF) file.
            tcp_frame (str): Name of the end-effector frame (MJCF site, body or joint).
            base_frame (str, optional): Frame poses are expressed in. Defaults to the world frame.
            dof (int, optional): Number of controlled joints (the first `dof` configuration variables). Defaults to all joints.
            format (ModelFormat, optional): File format. Defaults to AUTO.
            parameters (ClikParameters, optional): Solver parameters.
        """

    def forward(
        self,
        q: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]],
        tcp_offset: (
            numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]] | None
        ) = None,
    ) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
        """
        Forward kinematics.

        Args:
            q (np.ndarray): Joint configuration with `dof` or `nq` entries.
            tcp_offset (np.ndarray, optional): 4x4 offset applied to the tcp frame.

        Returns:
            np.ndarray: 4x4 pose of tcp_frame * tcp_offset in base_frame.
        """

    def frame_names(self) -> list[str]:
        """
        All frame names (bodies, joints, sites) of the model.
        """

    def inverse(
        self,
        pose: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]],
        q0: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]],
        tcp_offset: (
            numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]] | None
        ) = None,
    ) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]] | None:
        """
        Inverse kinematics (damped least squares CLIK).

        Args:
            pose (np.ndarray): Desired 4x4 pose of tcp_frame * tcp_offset in base_frame.
            q0 (np.ndarray): Initial guess with `dof` or `nq` entries.
            tcp_offset (np.ndarray, optional): 4x4 offset applied to the tcp frame.

        Returns:
            np.ndarray | None: Joint values of the `dof` controlled joints or None if the solver did not converge.
        """

    def joint_names(self) -> list[str]:
        """
        Joint names in configuration order.
        """

    def reference_configurations(self) -> dict[str, numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]]:
        """
        Named reference configurations (MJCF keyframes), restricted to the controlled joints.
        """

    @property
    def base_frame(self) -> str | None:
        """
        Name of the base frame or None for world.
        """

    @property
    def dof(self) -> int:
        """
        Number of controlled joints.
        """

    @property
    def model_name(self) -> str:
        """
        Name of the model as given in the file.
        """

    @property
    def nq(self) -> int:
        """
        Size of the full model configuration.
        """

    @property
    def nv(self) -> int:
        """
        Size of the full model velocity.
        """

    @property
    def parameters(self) -> ClikParameters:
        """
        Solver parameters (ClikParameters).
        """

    @parameters.setter
    def parameters(self, arg1: ClikParameters) -> None: ...
    @property
    def path(self) -> str:
        """
        Path of the loaded robot model.
        """

    @property
    def q_max(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]:
        """
        Upper position limits of the controlled joints.
        """

    @property
    def q_min(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]:
        """
        Lower position limits of the controlled joints.
        """

    @property
    def q_neutral(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]:
        """
        Neutral configuration of the controlled joints.
        """

    @property
    def q_rest(self) -> numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]:
        """
        Full (nq) configuration whose tail is used for the uncontrolled joints.
        """

    @q_rest.setter
    def q_rest(self, arg1: numpy.ndarray[tuple[M], numpy.dtype[numpy.float64]]) -> None: ...
    @property
    def tcp_frame(self) -> str:
        """
        Name of the end-effector frame.
        """

__version__: str = "1.0.0"
pinocchio_version: str = "3.7.0"
