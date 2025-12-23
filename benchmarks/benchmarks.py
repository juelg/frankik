import time
import typing

import genesis as gs
import matplotlib.pyplot as plt
import numpy as np
import rcs
import rcs_robotics_library
from ManipulaPy.urdf_processor import URDFToSerialManipulator

from frankik import FrankaKinematics


class Kinematics:
    def forward(
        self,
        q0: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]],
    ) -> np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]]:
        """Compute the forward kinematics for the given joint configuration.
        Args:
            q0 (np.ndarray): A 7-element array representing joint angles.
            tcp_offset (np.ndarray, optional): A 4x4 homogeneous transformation matrix representing
                the tool center point offset. Defaults to None.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix representing the end-effector pose.
        """
        raise NotImplementedError

    def inverse(
        self,
        pose: np.ndarray[tuple[typing.Literal[4], typing.Literal[4]], np.dtype[np.float64]],
        q0: np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]] | None = None,
    ) -> np.ndarray[tuple[typing.Literal[7]], np.dtype[np.float64]] | None:
        """Compute the inverse kinematics for the given end-effector pose.
        Args:
            pose (np.ndarray): A 4x4 homogeneous transformation matrix representing the desired end-effector pose.
            q0 (np.ndarray, optional): A 7-element array representing the current joint angles. Defaults to None.
            tcp_offset (np.ndarray, optional): A 4x4 homogeneous transformation matrix representing
                the tool center point offset. Defaults to None.
            allow_elbow_flips (bool): Whether to consider multiple IK solutions (elbow flips). Defaults to False.
            q7 (float): The angle of the seventh joint, used for FR3 robot IK. Defaults to π/4.
        Returns:
            np.ndarray | None: A 7-element array representing the joint angles if a solution is found; otherwise, None.
        """
        raise NotImplementedError


class FrankIK(Kinematics):
    def __init__(self):
        self.frankik = FrankaKinematics(robot_type="panda")

    def forward(self, q0):
        return self.frankik.forward(q0)

    def inverse(self, pose, q0=None):
        return self.frankik.inverse(pose, q0=q0, q7=np.pi/4)


class PinocchioCPP(Kinematics):
    def __init__(self):
        self.ik = rcs.common.Pin(
            "robot-control-stack/assets/scenes/fr3_empty_world/robot.xml",
            "attachment_site_0",
            urdf=False,
        )

    def forward(self, q0):
        return self.ik.forward(q0).pose_matrix()

    def inverse(self, pose, q0=None):
        pose = rcs.common.Pose(pose_matrix=pose)
        return self.ik.inverse(pose, q0=q0)


class RoboticsLibrary(Kinematics):
    def __init__(self):
        self.ik = rcs_robotics_library.rl.RoboticsLibraryIK("robot-control-stack/assets/fr3/urdf/fr3.urdf")

    def forward(self, q0):
        return self.ik.forward(q0).pose_matrix()

    def inverse(self, pose, q0=None):
        pose = rcs.common.Pose(pose_matrix=pose)
        return self.ik.inverse(pose, q0=q0)


class IKPy(Kinematics):
    def __init__(self):
        from ikpy.chain import Chain

        self.chain = Chain.from_urdf_file("robot-control-stack/assets/fr3/urdf/fr3.urdf", base_elements=["fr3_link0"])

    def forward(self, q0):
        q = np.zeros(9)
        q[1:8] = q0
        return self.chain.forward_kinematics(q)

    def inverse(self, pose, q0=None):
        q = np.zeros(9)
        q[1:8] = q0
        pose = rcs.common.Pose(pose_matrix=pose)
        ik_results = self.chain.inverse_kinematics(
            pose.translation(), initial_position=q
        )  # , pose.xyzrpy()[3:]) #, orientation_mode="all")
        if ik_results is None:
            return None
        return ik_results[1:8]  # Exclude the first element which is for the base


class FastIK(Kinematics):

    def forward(self, q0):
        from ikfast_franka_panda import get_fk

        trans, rot = get_fk(q0)
        pose = rcs.common.Pose(translation=np.array(trans), rotation=np.array(rot))
        return pose.pose_matrix()

    def inverse(self, pose, q0=None):
        from ikfast_franka_panda import get_ik

        pose = rcs.common.Pose(pose_matrix=pose)
        return get_ik(
            trans_list=pose.translation().tolist(), rot_list=pose.rotation_m().tolist(), free_jt_vals=[np.pi / 4]
        )


class RoboticstoolboxPython(Kinematics):
    def __init__(self):
        import roboticstoolbox as rtb

        self.robot = rtb.models.Panda()

    def forward(self, q0):
        return self.robot.fkine(q0).A

    def inverse(self, pose, q0=None):
        ik_results = self.robot.ik_LM(pose, q0=q0)
        return ik_results[0]


class ManipulaPy(Kinematics):
    def __init__(self):
        urdf_path = "robot-control-stack/assets/fr3/urdf/fr3.urdf"
        self.robot = URDFToSerialManipulator(urdf_path).serial_manipulator

    def forward(self, q0):
        return self.robot.forward_kinematics(q0)

    def inverse(self, pose, q0=None):
        ik_results = self.robot.iterative_inverse_kinematics(pose, q0)
        if ik_results is None:
            return None
        return ik_results


class GenesisWorld(Kinematics):
    def __init__(self):
        ########################## init ##########################
        gs.init(backend=gs.gpu)

        ########################## create a scene ##########################
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=0.01,
            ),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3, -1, 1.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=30,
                max_FPS=60,
            ),
            show_viewer=False,
        )

        ########################## entities ##########################
        plane = self.scene.add_entity(
            gs.morphs.Plane(),
        )
        cube = self.scene.add_entity(
            gs.morphs.Box(
                size=(0.04, 0.04, 0.04),
                pos=(0.65, 0.0, 0.02),
            )
        )
        self.franka = self.scene.add_entity(
            gs.morphs.MJCF(file="mujoco_menagerie/franka_emika_panda/panda.xml"),
        )
        ########################## build ##########################
        self.scene.build()
        motors_dof = np.arange(7)
        fingers_dof = np.arange(7, 9)

        # set control gains
        # Note: the following values are tuned for achieving best behavior with Franka
        # Typically, each new robot would have a different set of parameters.
        # Sometimes high-quality URDF or XML file would also provide this and will be parsed.
        self.franka.set_dofs_kp(
            np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
        )
        self.franka.set_dofs_kv(
            np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
        )
        self.franka.set_dofs_force_range(
            np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
            np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
        )
        self.end_effector = self.franka.get_link("hand")  # hand_0, attachment_site_0

    def inverse(self, pose, q0=None):
        pose = rcs.common.Pose(pose_matrix=pose)
        qpos = self.franka.inverse_kinematics(
            link=self.end_effector,
            pos=pose.translation(),
            quat=pose.rotation_q(),
        )
        return qpos

    def forward(self, q0):
        q = np.zeros(9)
        q[:7] = q0

        # self.franka.set_dofs_position(q)
        # self.scene.step()  # update forward kinematics
        self.franka.set_qpos(q)
        q = self.end_effector.get_quat().cpu().numpy().copy()
        q2 = q.copy()
        q[0] = q[3]
        q[3] = q2[0]
        pose = rcs.common.Pose(translation=self.end_effector.get_pos().cpu().numpy(), quaternion=q)

        return pose.pose_matrix()


def test_speed(kinematics: Kinematics, n_iters: int = 1000):
    """
    Benchmarks FK and IK speed for small and large perturbations around home pose.

    Returns:
        tuple: (avg_fk_small, avg_ik_small, avg_fk_large, avg_ik_large) in seconds.
    """
    # 1. Setup Data and Seeds
    np.random.seed(42)  # Ensure reproducibility

    # We instantiate FrankaKinematics just to get the robot limits/constants
    # independent of the library being tested.
    ref_robot = FrankaKinematics(robot_type="panda")
    q_home = ref_robot.q_home
    q_min = ref_robot.q_min
    q_max = ref_robot.q_max

    # Convert degrees to radians
    deg_3 = np.deg2rad(3.0)
    deg_10 = np.deg2rad(10.0)

    # Timers
    t_fk_small_acc = 0.0
    t_ik_small_acc = 0.0
    t_fk_large_acc = 0.0
    t_ik_large_acc = 0.0

    for _ in range(n_iters):
        # --- Category 1: Small Perturbation (max 3 deg) ---
        noise_small = np.random.uniform(-deg_3, deg_3, size=7)
        q_target_small = np.clip(q_home + noise_small, q_min, q_max)

        # Measure FK (Small)
        start = time.perf_counter()
        pose_small = kinematics.forward(q_target_small)
        t_fk_small_acc += time.perf_counter() - start

        # Measure IK (Small) - using q_home as seed
        start = time.perf_counter()
        _ = kinematics.inverse(pose_small, q0=q_home)
        t_ik_small_acc += time.perf_counter() - start

        # --- Category 2: Large Perturbation (10 deg) ---
        noise_large = np.random.uniform(-deg_10, deg_10, size=7)
        q_target_large = np.clip(q_home + noise_large, q_min, q_max)

        # Measure FK (Large)
        start = time.perf_counter()
        pose_large = kinematics.forward(q_target_large)
        t_fk_large_acc += time.perf_counter() - start

        # Measure IK (Large) - using q_home as seed
        start = time.perf_counter()
        _ = kinematics.inverse(pose_large, q0=q_home)
        t_ik_large_acc += time.perf_counter() - start

    # Calculate averages
    return (t_fk_small_acc / n_iters, t_ik_small_acc / n_iters, t_fk_large_acc / n_iters, t_ik_large_acc / n_iters)


def benchmark_all():
    kinematics_classes = [
        (FrankIK, 1000),
        (RoboticstoolboxPython, 1000),  # numpy version needs to be below 2.0
        (FastIK, 1000),
        (PinocchioCPP, 1000),
        (RoboticsLibrary, 1000),
        (ManipulaPy, 1000),
        (GenesisWorld, 1000),
        (IKPy, 100),
    ]

    benchmark_data = []

    print("Running benchmarks... ")

    for cls, n_iters in kinematics_classes:
        library_name = cls.__name__

        # We wrap the execution in the suppressor to hide the "crap"
        # printed during init or runtime
        # try:
        # 1. Measure Init
        t0 = time.perf_counter()
        instance = cls()
        t_init = time.perf_counter() - t0

        # 2. Measure Speed
        results = test_speed(instance, n_iters=n_iters)

        # Store success
        benchmark_data.append({"name": library_name, "status": "OK", "t_init": t_init, "results": results})

        # except Exception as e:
        #     # Store failure
        #     benchmark_data.append({"name": library_name, "status": "FAIL", "error": str(e)})

    # --- Print Table After All Runs ---
    print("\n" + "=" * 95)
    print(
        f"{'Library':<25} | {'Init (s)':<10} | {'FK-Small(s)':<12} | {'IK-Small(s)':<12} | {'FK-Large(s)':<12} | {'IK-Large(s)':<12}"
    )
    print("-" * 95)

    for entry in benchmark_data:
        if entry["status"] == "OK":
            fk_s, ik_s, fk_l, ik_l = entry["results"]
            print(
                f"{entry['name']:<25} | {entry['t_init']:<10.5f} | {fk_s:<12.7f} | {ik_s:<12.7f} | {fk_l:<12.7f} | {ik_l:<12.7f}"
            )
        else:
            print(f"{entry['name']:<25} | FAILED: {entry['error']}")
    print("=" * 95 + "\n")

    plot_results(benchmark_data)


def plot_results(benchmark_data):
    # Filter out failed benchmarks
    successful_benchmarks = [entry for entry in benchmark_data if entry["status"] == "OK"]

    if not successful_benchmarks:
        print("No successful benchmarks to plot.")
        return

    library_names = [entry["name"] for entry in successful_benchmarks]
    ik_small_times = [entry["results"][1] for entry in successful_benchmarks]
    ik_large_times = [entry["results"][3] for entry in successful_benchmarks]

    n_libraries = len(library_names)
    bar_width = 0.35
    index = np.arange(n_libraries)

    # Create a single plot for Inverse Kinematics
    fig, ax = plt.subplots(figsize=(10, 6))
    # fig.suptitle("Inverse Kinematics Benchmark Results", fontsize=16)

    # Plot IK times
    ax.bar(index, ik_small_times)
    # ax.bar(index - bar_width/2, ik_small_times, bar_width, label="IK Small Perturbation")
    # ax.bar(index + bar_width/2, ik_large_times, bar_width, label="IK Large Perturbation")

    ax.set_ylabel("Time (s) - Log Scale")
    ax.set_title("Inverse Kinematics Speed")
    ax.set_xticks(index)
    ax.set_xticklabels(library_names, rotation=45, ha="right")
    ax.set_yscale("log")  # Set y-axis to logarithmic scale
    ax.legend()
    ax.grid(axis="y", linestyle="--", alpha=0.7)

    plt.savefig("benchmark_results.svg", format="svg", dpi=300, bbox_inches="tight")


if __name__ == "__main__":
    benchmark_all()
