import numpy as np
import pytest

from frankik import FrankaKinematics, RobotType


@pytest.mark.parametrize("robot_type", [RobotType.PANDA, RobotType.FR3])
def test_example(robot_type):
    kinematics = FrankaKinematics(robot_type=robot_type)
    q_home = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])
    pose_home = kinematics.forward(q_home, tcp_offset=kinematics.FrankaHandTCPOffset)  # type: ignore
    q = kinematics.inverse(pose_home, tcp_offset=kinematics.FrankaHandTCPOffset, q0=q_home)  # type: ignore
    assert np.allclose(q, q_home)  # type: ignore
