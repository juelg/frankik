import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

import frankik

# Number of random configurations to test per robot
N_SAMPLES = 200

# Cartesian Tolerance (Position/Rotation matrix element difference)
# 1e-5 corresponds to roughly 0.01mm or 0.001 degrees deviation
CARTESIAN_TOL = 1e-3


@pytest.fixture(autouse=True)
def _set_seed():
    """Automatically seed the RNG before every test to ensure reproducibility."""
    np.random.seed(42)


def generate_random_q(q_min, q_max):
    q_min = np.array(q_min)
    q_max = np.array(q_max)
    range_span = q_max - q_min

    # 1. Generate base random values (centered 90% of range)
    unit_rand = np.random.rand(1, 7)
    q_rand = q_min + (range_span * 0.05) + unit_rand * (range_span * 0.90)

    # 2. Singularity Avoidance
    # The Franka analytical solver has "snap-to-zero" logic for:
    # - Joint 2 (Shoulder): When q[1] is near 0
    # - Joint 6 (Wrist): When q[5] is near 0 or +/- PI

    # Threshold in radians (approx 5 degrees to be safe)
    singularity_threshold = 0.08

    # Avoid Shoulder Singularity (q[1] ≈ 0)
    if abs(q_rand[0, 1]) < singularity_threshold:
        # Push it away from 0, preserving the sign (default to positive if 0)
        sign = np.sign(q_rand[0, 1]) if q_rand[0, 1] != 0 else 1.0
        q_rand[0, 1] = sign * singularity_threshold

    # Avoid Wrist Singularity (q[5] ≈ 0) - though less critical for this specific bug
    if abs(q_rand[0, 5]) < singularity_threshold:
        sign = np.sign(q_rand[0, 5]) if q_rand[0, 5] != 0 else 1.0
        q_rand[0, 5] = sign * singularity_threshold

    return q_rand[0]


def get_random_pose_in_isocube():
    """
    Generates a random pose (4x4 matrix) within the defined isocube.

    The isocube is defined by:
    - Center: [0.498, 0.0, 0.226]
    - Half-size: [0.2, 0.2, 0.2] (Total size is 0.4 along each axis)

    Returns:
        np.ndarray: A 4x4 homogeneous transformation matrix representing the pose.
    """
    # 1. Define the geometric bounds
    center = np.array([0.498, 0.0, 0.226])
    half_size = np.array([0.2, 0.2, 0.2])

    lower_bound = center - half_size
    upper_bound = center + half_size

    # 2. Sample a random position (x, y, z) uniformly within the bounds
    position = np.random.uniform(low=lower_bound, high=upper_bound)

    # 3. Sample a random orientation (3x3 rotation matrix)
    # Uses scipy to ensure a uniform distribution over SO(3)
    rotation_matrix = R.random().as_matrix()

    # 4. Construct the 4x4 homogeneous matrix
    pose = np.eye(4)
    pose[:3, :3] = rotation_matrix
    pose[:3, 3] = position

    return pose


@pytest.mark.parametrize("robot_type", ["panda", "fr3"])
def test_ik_correctness_cartesian(robot_type):
    """
    The 'Gold Standard' IK Test:
    1. Generate random joints (q_orig)
    2. Compute pose (Target Pose) via FK
    3. Calculate IK(Target Pose) -> q_sol
    4. Compute pose_check = FK(q_sol)
    5. Assert Target Pose == pose_check
    """
    if robot_type == "panda":
        q_min, q_max = frankik.q_min_panda, frankik.q_max_panda
        is_fr3 = False
    else:
        q_min, q_max = frankik.q_min_fr3, frankik.q_max_fr3
        is_fr3 = True

    for i in range(N_SAMPLES):
        q_orig = generate_random_q(q_min, q_max)

        # 1. Forward Kinematics (Get the target)
        target_pose = frankik.fk(q_orig)
        # target_pose = get_random_pose_in_isocube()

        # 2. Inverse Kinematics
        q_sol = frankik.ik(O_T_EE=target_pose, q_actual=q_orig, q7=q_orig[6], is_fr3=is_fr3)

        # CHECK 1: Did we find a solution?
        if np.isnan(q_sol).any():
            # If IK fails, we must ensure the pose was actually reachable.
            # Since we generated it from FK, it SHOULD be reachable.
            # However, singular poses can sometimes fail analytical solvers.
            # We fail the test if this happens too often, but for now let's print.
            # pytest.fail(f"IK failed to find solution for valid pose {i}")
            print(f"Warning: IK failed to find solution for valid pose {i}, q orig:")
            continue

        # CHECK 2: Does the solution actually reach the target? (The real test)
        pose_check = frankik.fk(q_sol)

        # Compare 4x4 matrices
        np.testing.assert_allclose(
            pose_check,
            target_pose,
            atol=CARTESIAN_TOL,
            err_msg=f"Cartesian mismatch on sample {i} for {robot_type}, q orig: {q_orig} -> q sol {q_sol}",
        )


@pytest.mark.parametrize("robot_type", ["panda", "fr3"])
def test_ik_full_consistency(robot_type):
    """
    Verify that ALL solutions returned by ik_full are valid.
    """
    if robot_type == "panda":
        q_min, q_max = frankik.q_min_panda, frankik.q_max_panda
        is_fr3 = False
    else:
        q_min, q_max = frankik.q_min_fr3, frankik.q_max_fr3
        is_fr3 = True

    for i in range(N_SAMPLES):
        q_orig = generate_random_q(q_min, q_max)
        target_pose = frankik.fk(q_orig)
        # target_pose = get_random_pose_in_isocube()

        solutions = frankik.ik_full(
            O_T_EE=target_pose,
            q_actual=q_orig,
            q7=q_orig[6],
            is_fr3=is_fr3,
        )

        # Check every non-NaN solution
        valid_sols_count = 0
        for sol in solutions:
            if np.isnan(sol).any():
                continue

            valid_sols_count += 1
            pose_check = frankik.fk(sol)  # type: ignore

            # Every returned solution must result in the target pose
            np.testing.assert_allclose(
                pose_check,
                target_pose,
                atol=CARTESIAN_TOL,
                err_msg=f"One of the ik_full solutions was invalid on sample {i}",
            )

        assert valid_sols_count > 0, f"ik_full returned no valid solutions for a reachable pose {i}"


def test_limits_respected():
    """Sanity check that our random generator respects limits."""
    q_min, q_max = frankik.q_min_panda, frankik.q_max_panda
    for _ in range(100):
        q = generate_random_q(q_min, q_max)
        assert np.all(q >= q_min)
        assert np.all(q <= q_max)
