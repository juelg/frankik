import os
from pathlib import Path

import numpy as np
import pytest

import frankik
from frankik import FrankaKinematics, Kinematics, PinocchioKinematics, RobotType

pytestmark = pytest.mark.skipif(
    not frankik.has_pinocchio() and not os.environ.get("FRANKIK_REQUIRE_PINOCCHIO"),
    reason="pinocchio is not installed (pip install 'frankik[pinocchio]')",
)

N_SAMPLES = 50
CARTESIAN_TOL = 1e-3


@pytest.fixture(autouse=True)
def _set_seed():
    np.random.seed(42)


def random_q(q_min: np.ndarray, q_max: np.ndarray, margin: float = 0.1) -> np.ndarray:
    span = q_max - q_min
    return q_min + margin * span + np.random.rand(len(q_min)) * (1 - 2 * margin) * span  # type: ignore


def assert_pose_close(a: np.ndarray, b: np.ndarray, atol: float = CARTESIAN_TOL, msg: str = ""):
    np.testing.assert_allclose(a, b, atol=atol, err_msg=msg)


@pytest.mark.parametrize("robot_type", list(RobotType))
def test_bundled_model_description(robot_type):
    description = robot_type.description
    assert description.mjcf_path.exists(), description.mjcf_path
    assert robot_type.mjcf_path == description.mjcf_path
    assert description.dof == 7
    assert description.q_home.shape == (7,)


@pytest.mark.parametrize("robot_type", list(RobotType))
def test_bundled_model_loads(robot_type):
    kin = PinocchioKinematics(robot_type)
    assert isinstance(kin, Kinematics)
    assert kin.robot_type == robot_type
    assert kin.dof == 7
    assert kin.nq == 7
    assert len(kin.joint_names) == 7
    assert kin.tcp_frame == "attachment_site"
    assert kin.tcp_frame in kin.frame_names
    assert kin.base_frame is not None
    assert kin.base_frame in kin.frame_names
    assert kin.q_min.shape == (7,)
    assert np.all(kin.q_min < kin.q_max)
    assert "home" in kin.reference_configurations
    assert kin.reference_configurations["home"].shape == (7,)
    np.testing.assert_allclose(kin.q_home, frankik.Q_HOME_FRANKA)
    assert robot_type.value in repr(kin)


def test_robot_type_from_string():
    kin = PinocchioKinematics("fr3")
    assert kin.robot_type == RobotType.FR3
    assert FrankaKinematics("panda").robot_type == RobotType.PANDA
    assert RobotType.FR3 == "fr3"


@pytest.mark.parametrize("robot_type", list(RobotType))
def test_forward_matches_analytical_solver(robot_type):
    """The bundled attachment_site is the flange, the analytical solver returns flange * FrankaHandTCPOffset."""
    kin = PinocchioKinematics(robot_type)
    ana = FrankaKinematics(robot_type)
    q_min = np.maximum(kin.q_min, ana.q_min)
    q_max = np.minimum(kin.q_max, ana.q_max)
    for _ in range(N_SAMPLES):
        q = random_q(q_min, q_max)
        pose_pin = kin.forward(q, tcp_offset=FrankaKinematics.FrankaHandTCPOffset)
        pose_ana = ana.forward(q)
        assert_pose_close(pose_pin, pose_ana, atol=1e-3, msg=f"q={q}")


@pytest.mark.parametrize("robot_type", list(RobotType))
def test_inverse_roundtrip(robot_type):
    kin = PinocchioKinematics(robot_type)
    failures = 0
    for i in range(N_SAMPLES):
        q = random_q(kin.q_min, kin.q_max)
        target = kin.forward(q)
        q0 = q + np.random.uniform(-0.2, 0.2, size=kin.dof)
        q_sol = kin.inverse(target, q0=q0)
        if q_sol is None:
            failures += 1
            continue
        assert q_sol.shape == (kin.dof,)
        assert_pose_close(kin.forward(q_sol), target, msg=f"sample {i}, q={q}")
    assert failures == 0, f"IK did not converge for {failures}/{N_SAMPLES} reachable poses"


@pytest.mark.parametrize("robot_type", list(RobotType))
def test_inverse_with_tcp_offset(robot_type):
    kin = PinocchioKinematics(robot_type)
    tcp = FrankaKinematics.FrankaHandTCPOffset
    for _ in range(10):
        q = random_q(kin.q_min, kin.q_max)
        target = kin.forward(q, tcp_offset=tcp)
        q_sol = kin.inverse(target, q0=kin.q_home, tcp_offset=tcp)
        assert q_sol is not None
        assert_pose_close(kin.forward(q_sol, tcp_offset=tcp), target)


def test_inverse_default_seed_is_q_home():
    kin = PinocchioKinematics(RobotType.FR3)
    target = kin.forward(kin.q_home)
    q_sol = kin.inverse(target)
    assert q_sol is not None
    np.testing.assert_allclose(q_sol, kin.q_home, atol=1e-3)


def test_inverse_unreachable_returns_none():
    kin = PinocchioKinematics(RobotType.FR3, max_iterations=100)
    target = np.eye(4)
    target[:3, 3] = [3.0, 0.0, 0.0]
    assert kin.inverse(target) is None


def test_solver_parameters():
    kin = PinocchioKinematics(RobotType.FR3, eps=1e-6, max_iterations=500)
    assert kin.parameters.eps == 1e-6
    assert kin.parameters.max_iterations == 500
    assert kin.parameters.clamp_joint_limits is True
    assert kin.parameters.dt == 0.1
    assert kin.parameters.restarts == 10
    assert PinocchioKinematics(RobotType.FR3, clamp_joint_limits=False).parameters.clamp_joint_limits is False

    params = kin.parameters
    params.damping = 1e-3
    params.max_iterations = 1000
    kin.parameters = params
    assert kin.parameters.damping == 1e-3
    assert kin.parameters.max_iterations == 1000

    for _ in range(10):
        q = random_q(kin.q_min, kin.q_max)
        target = kin.forward(q)
        q_sol = kin.inverse(target, q0=q + np.random.uniform(-0.2, 0.2, size=kin.dof))
        assert q_sol is not None
        assert np.all(q_sol >= kin.q_min - 1e-9) and np.all(q_sol <= kin.q_max + 1e-9)
        assert_pose_close(kin.forward(q_sol), target, atol=1e-5)

    with pytest.raises(TypeError):
        PinocchioKinematics(RobotType.FR3, foo=1)
    with pytest.raises(TypeError, match="either"):
        PinocchioKinematics(RobotType.FR3, parameters=params, eps=1e-3)


def test_q_home_override():
    q_home = np.zeros(7)
    kin = PinocchioKinematics(RobotType.PANDA, q_home=q_home)
    np.testing.assert_allclose(kin.q_home, q_home)
    kin.q_home = frankik.Q_HOME_FRANKA
    np.testing.assert_allclose(kin.q_home, frankik.Q_HOME_FRANKA)


@pytest.fixture()
def mounted_fr3_with_finger(tmp_path: Path) -> Path:
    """FR3 mounted with an offset below an intermediate body (Pinocchio ignores the root body pose) plus an
    uncontrolled slide joint."""
    xml = RobotType.FR3.mjcf_path.read_text()
    mount = (
        '<body name="table">\n'
        '<body name="mount" pos="0.5 0.2 0.1" quat="0.7071068 0 0 0.7071068">\n'
        '<body name="base" childclass="fr3">'
    )
    assert '<body name="base" childclass="fr3">' in xml
    xml = xml.replace('<body name="base" childclass="fr3">', mount)
    xml = xml.replace("</worldbody>", "</body>\n</body>\n  </worldbody>")
    finger = (
        '<site name="attachment_site" pos="0 0 0.107"/>\n'
        '<body name="finger" pos="0 0 0.15">\n'
        '  <inertial pos="0 0 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>\n'
        '  <joint name="finger_joint" type="slide" axis="0 1 0" range="0 0.04"/>\n'
        '  <site name="finger_tip" pos="0 0 0.02"/>\n'
        "</body>"
    )
    assert '<site name="attachment_site" pos="0 0 0.107"/>' in xml
    xml = xml.replace('<site name="attachment_site" pos="0 0 0.107"/>', finger)
    xml = xml.replace(
        'qpos="0 0 0 -1.57079 0 1.57079 -0.7853" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853"',
        'qpos="0 0 0 -1.57079 0 1.57079 -0.7853 0.01" ctrl="0 0 0 -1.57079 0 1.57079 -0.7853"',
    )
    path = tmp_path / "mounted_fr3.xml"
    path.write_text(xml)
    return path


def test_custom_mjcf_requires_tcp_frame(mounted_fr3_with_finger):
    with pytest.raises(ValueError, match="tcp_frame"):
        PinocchioKinematics(mounted_fr3_with_finger)


def test_custom_mjcf_dof_and_base_frame(mounted_fr3_with_finger):
    kin = PinocchioKinematics(mounted_fr3_with_finger, tcp_frame="attachment_site", base_frame="base", dof=7)
    reference = PinocchioKinematics(RobotType.FR3)
    assert kin.robot_type is None
    assert kin.nq == 8
    assert kin.dof == 7
    assert kin.joint_names[-1] == "finger_joint"
    assert kin.q_min.shape == (7,)
    assert kin.reference_configurations["home"].shape == (7,)
    np.testing.assert_allclose(kin.q_home, kin.reference_configurations["home"])
    assert kin.path == mounted_fr3_with_finger

    for _ in range(10):
        q = random_q(kin.q_min, kin.q_max)
        assert_pose_close(kin.forward(q), reference.forward(q), atol=1e-9)
        assert_pose_close(kin.forward(np.append(q, 0.02)), reference.forward(q), atol=1e-9)
        q_sol = kin.inverse(reference.forward(q), q0=kin.q_home)
        assert q_sol is not None
        assert q_sol.shape == (7,)
        assert_pose_close(kin.forward(q_sol), reference.forward(q))


def test_custom_mjcf_world_frame(mounted_fr3_with_finger):
    kin = PinocchioKinematics(mounted_fr3_with_finger, tcp_frame="attachment_site", dof=7)
    reference = PinocchioKinematics(RobotType.FR3)
    assert kin.base_frame is None
    mount = np.eye(4)
    mount[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    mount[:3, 3] = [0.5, 0.2, 0.1]
    q = random_q(kin.q_min, kin.q_max)
    assert_pose_close(kin.forward(q), mount @ reference.forward(q), atol=1e-6)
    q_sol = kin.inverse(mount @ reference.forward(q), q0=kin.q_home)
    assert q_sol is not None
    assert_pose_close(kin.forward(q_sol), mount @ reference.forward(q))


def test_custom_mjcf_uncontrolled_joint_and_q_rest(mounted_fr3_with_finger):
    kin = PinocchioKinematics(mounted_fr3_with_finger, tcp_frame="finger_tip", base_frame="base", dof=7)
    q = random_q(kin.q_min, kin.q_max)
    pose_closed = kin.forward(q)
    q_rest = np.zeros(8)
    q_rest[7] = 0.04
    kin.q_rest = q_rest
    np.testing.assert_allclose(kin.q_rest, q_rest)
    pose_open = kin.forward(q)
    assert not np.allclose(pose_closed, pose_open)
    np.testing.assert_allclose(np.linalg.norm(pose_open[:3, 3] - pose_closed[:3, 3]), 0.04, atol=1e-9)
    q_sol = kin.inverse(pose_open, q0=kin.q_home)
    assert q_sol is not None
    assert q_sol.shape == (7,)
    assert_pose_close(kin.forward(q_sol), pose_open)

    with pytest.raises(ValueError, match="q_rest"):
        kin.q_rest = np.zeros(3)


def test_body_as_tcp_frame(mounted_fr3_with_finger):
    kin = PinocchioKinematics(mounted_fr3_with_finger, tcp_frame="fr3_link7", base_frame="base", dof=7)
    site = PinocchioKinematics(mounted_fr3_with_finger, tcp_frame="attachment_site", base_frame="base", dof=7)
    q = random_q(kin.q_min, kin.q_max)
    flange_offset = np.eye(4)
    flange_offset[2, 3] = 0.107
    assert_pose_close(kin.forward(q, tcp_offset=flange_offset), site.forward(q), atol=1e-9)


FR3_URDF = Path(__file__).parent / "data" / "fr3.urdf"


def test_urdf_matches_mjcf():
    urdf = PinocchioKinematics(FR3_URDF, tcp_frame="fr3_link8", base_frame="fr3_link0")
    mjcf = PinocchioKinematics(RobotType.FR3)
    assert urdf.dof == 7
    assert urdf.joint_names == mjcf.joint_names
    np.testing.assert_allclose(urdf.q_min, frankik.q_min_fr3)
    np.testing.assert_allclose(urdf.q_max, frankik.q_max_fr3)
    for _ in range(N_SAMPLES):
        q = random_q(urdf.q_min, urdf.q_max)
        target = mjcf.forward(q)
        assert_pose_close(urdf.forward(q), target, atol=1e-9)
        q_sol = urdf.inverse(target, q0=q + np.random.uniform(-0.2, 0.2, size=7))
        assert q_sol is not None
        assert_pose_close(urdf.forward(q_sol), target)


def test_model_format_selection():
    auto = PinocchioKinematics(FR3_URDF, tcp_frame="fr3_link8")
    explicit = PinocchioKinematics(str(FR3_URDF), tcp_frame="fr3_link8", model_format="urdf")
    q = random_q(auto.q_min, auto.q_max)
    assert_pose_close(auto.forward(q), explicit.forward(q), atol=1e-12)
    with pytest.raises(ValueError, match="Unknown model format"):
        PinocchioKinematics(FR3_URDF, tcp_frame="fr3_link8", model_format="sdf")


def test_errors():
    with pytest.raises(ValueError, match="does not exist"):
        PinocchioKinematics("/does/not/exist.xml", tcp_frame="x")
    with pytest.raises(ValueError, match="attachment_site"):
        PinocchioKinematics(RobotType.FR3, tcp_frame="not_a_frame")
    with pytest.raises(ValueError, match="base_frame"):
        PinocchioKinematics(RobotType.FR3, base_frame="not_a_frame")
    with pytest.raises(ValueError, match="dof"):
        PinocchioKinematics(RobotType.FR3, dof=8)
    kin = PinocchioKinematics(RobotType.FR3)
    with pytest.raises(ValueError, match="dof=7"):
        kin.forward(np.zeros(3))
    with pytest.raises(TypeError, match="q7"):
        kin.inverse(np.eye(4), q7=0.0)  # type: ignore[call-arg]
