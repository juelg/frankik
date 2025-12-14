import numpy as np
import pytest

from frankik import FrankaKinematics, RobotType


@pytest.mark.parametrize("robot_type", [RobotType.PANDA, RobotType.FR3])
def test_example(robot_type):
    kinematics = FrankaKinematics(robot_type=robot_type)
    q_home = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])
    pose_home = kinematics.forward(q_home)
    q = kinematics.inverse(pose_home)
    assert np.allclose(q, q_home)
