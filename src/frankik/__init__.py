import typing

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


class RobotType:
    PANDA = "panda"
    FR3 = "fr3"


class FrankaKinematics:
    FrankaHandTCPOffset = np.array(
        [
            [0.707, 0.707, 0.0, 0.0],
            [-0.707, 0.707, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.1034],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    def __init__(self, robot_type: str = RobotType.FR3):
        """Initialize Franka Kinematics for the specified robot type.
        Args:
            robot_type (str): Type of the robot, either 'panda' or 'fr3'.
        Raises:
            ValueError: If an unsupported robot type is provided.
        """
        if robot_type not in (RobotType.PANDA, RobotType.FR3):
            msg = f"Unsupported robot type: {robot_type}. Choose 'panda' or 'fr3'."
            raise ValueError(msg)
        self.robot_type = robot_type
        self.q_min = q_min_fr3 if robot_type == RobotType.FR3 else q_min_panda
        self.q_max = q_max_fr3 if robot_type == RobotType.FR3 else q_max_panda
        self.q_home = kQDefault

    @staticmethod
    def pose_inverse(
        T: np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]]
    ) -> np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]]:
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
        return T_inv  # type: ignore

    def forward(
        self,
        q0: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]],
        tcp_offset: np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]] | None = None,
    ) -> np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]]:
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
        return pose @ self.FrankaHandTCPOffset @ self.pose_inverse(tcp_offset) if tcp_offset is not None else pose  # type: ignore

    def inverse(
        self,
        pose: np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]],
        q0: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]] | None = None,
        tcp_offset: np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]] | None = None,
        q7: float | None = None,
    ) -> np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]] | None:
        """Compute the inverse kinematics for the given end-effector pose.

        Args:
            pose (np.ndarray): A 4x4 homogeneous transformation matrix representing the desired end-effector pose.
            q0 (np.ndarray, optional): A 7-element array representing the current joint angles. Defaults to None.
            tcp_offset (np.ndarray, optional): A 4x4 homogeneous transformation matrix representing
                the tool center point offset. Defaults to None.
            q7 (float, optional): The angle of the seventh joint, used for FR3 robot IK. If None then it will be sampled. Defaults to None.

        Returns:
            np.ndarray | None: A 7-element array representing the joint angles if a solution is found; otherwise, None.
        """
        new_pose = pose @ self.pose_inverse(tcp_offset) if tcp_offset is not None else pose
        if q7 is not None:
            q = ik(new_pose, q0, q7, is_fr3=(self.robot_type == RobotType.FR3))  # type: ignore
        else:
            q = ik_sample_q7(new_pose, q0, is_fr3=(self.robot_type == RobotType.FR3))  # type: ignore

        if np.isnan(q).any():
            return None
        return q


__all__ = [
    "__version__",
    "ik",
    "ik_full",
    "fk",
    "q_max_fr3",
    "q_max_panda",
    "q_min_fr3",
    "q_min_panda",
    "FrankaKinematics",
    "RobotType",
]
