# ATTENTION: auto generated from C++ code, use `make stubgen` to update!
"""
Python bindings for frankik IK/FK
"""
from __future__ import annotations

import typing

import numpy

__all__: list[str] = [
    "fk",
    "ik",
    "ik_full",
    "ik_sample_q7",
    "kQDefault",
    "q_max_fr3",
    "q_max_panda",
    "q_min_fr3",
    "q_min_panda",
]

def fk(
    q: numpy.ndarray[tuple[typing.Literal[7]], numpy.dtype[numpy.float64]]
) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]]:
    """
    Compute forward kinematics for Franka robot.

    Args:
        q (Vector7d): Joint angles.

    Returns:
        Eigen::Matrix<double, 4, 4>: End-effector pose.
    """

def ik(
    O_T_EE: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]],
    q_actual: numpy.ndarray[tuple[typing.Literal[7]], numpy.dtype[numpy.float64]] | None = None,
    q7: float = 0.7853981633974483,
    is_fr3: bool = False,
) -> numpy.ndarray[tuple[typing.Literal[7]], numpy.dtype[numpy.float64]]:
    """
    Compute one inverse kinematics solution for Franka robot.

    Args:
        O_T_EE (Eigen::Matrix<double, 4, 4>): Desired end-effector pose.
        q_actual (Vector7d, optional): Current joint angles. Defaults to kQDefault.
        q7 (double, optional): Joint 7 angle. Defaults to M_PI_4.
        is_fr3 (bool, optional): Whether to use FR3 joint limits. Defaults to false.

    Returns:
        Vector7d: One IK solution. NaN if no solution.
    """

def ik_full(
    O_T_EE: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]],
    q_actual: numpy.ndarray[tuple[typing.Literal[7]], numpy.dtype[numpy.float64]] | None = None,
    q7: float = 0.7853981633974483,
    is_fr3: bool = False,
) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[7]], numpy.dtype[numpy.float64]]:
    """
    Compute full inverse kinematics for Franka robot.

    Args:
        O_T_EE (Eigen::Matrix<double, 4, 4>): Desired end-effector pose.
        q_actual (Vector7d, optional): Current joint angles. Defaults to kQDefault.
        q7 (double, optional): Joint 7 angle. Defaults to M_PI_4.
        is_fr3 (bool, optional): Whether to use FR3 joint limits. Defaults to false.

    Returns:
        Eigen::Matrix<double, 4, 7>: All possible IK solutions (up to 4). NaN if no solution.
    """

def ik_sample_q7(
    O_T_EE: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]],
    q_actual: numpy.ndarray[tuple[typing.Literal[7]], numpy.dtype[numpy.float64]] | None = None,
    is_fr3: bool = False,
    sample_size: int = 60,
    sample_interval: float = 40,
    full_ik: bool = False,
) -> list[numpy.ndarray[tuple[typing.Literal[7]], numpy.dtype[numpy.float64]]]:
    """
    Compute one inverse kinematics solution for Franka with sampling of joint q7.

    Args:
        O_T_EE (np.array): Desired end-effector pose. Shape (4, 4).
        q_actual (np.array, optional): Current joint angles. Shape (7,).

        is_fr3 (bool, optional): Whether to use FR3 joint limits. Defaults to False.
        sample_size (int, optional): How many sample to try for q7. Defaults to 20.
        sample_interval (int, optional): Sample interval for q7 in degree. Defaults to 90.
        full_ik (bool, optional): Whether to use full IK. Defaults to False.
    degree. Defaults to False.

    Returns:
        list[np.array]: One IK solution. Empty if no solution was found. Array shape (7,).
    """

__version__: str = "0.1"
kQDefault: numpy.ndarray  # value = array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        ,...
q_max_fr3: list = [2.3093, 1.5133, 2.4937, -0.4461, 2.48, 4.2094, 2.6895]
q_max_panda: list = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
q_min_fr3: list = [-2.3093, -1.5133, -2.4937, -2.7478, -2.48, 0.8521, -2.6895]
q_min_panda: list = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
