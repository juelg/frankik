#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "frankik.h"

// TODO: define exceptions

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  m.doc() = "Python bindings for frankik IK/FK";
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  m.attr("q_min_panda") = frankik::q_min_panda;
  m.attr("q_max_panda") = frankik::q_max_panda;
  m.attr("q_min_fr3") = frankik::q_min_fr3;
  m.attr("q_max_fr3") = frankik::q_max_fr3;
  m.attr("kQDefault") = frankik::kQDefault;

  m.def("ik_full", &frankik::ik_full, py::arg("O_T_EE"),
        py::arg("q_actual") = std::nullopt, py::arg("q7") = M_PI_4,
        py::arg("is_fr3") = false,
        "Compute full inverse kinematics for Franka robot.\n\n"
        "Args:\n"
        "    O_T_EE (Eigen::Matrix<double, 4, 4>): Desired end-effector pose.\n"
        "    q_actual (Vector7d, optional): Current joint angles. "
        "Defaults to kQDefault.\n"
        "    q7 (double, optional): Joint 7 angle. Defaults to M_PI_4.\n"
        "    is_fr3 (bool, optional): Whether to use FR3 joint limits. "
        "Defaults to false.\n\n"
        "Returns:\n"
        "    Eigen::Matrix<double, 4, 7>: All possible IK solutions (up to 4). "
        "NaN if no solution.");
  m.def("ik", &frankik::ik, py::arg("O_T_EE"),
        py::arg("q_actual") = std::nullopt, py::arg("q7") = M_PI_4,
        py::arg("is_fr3") = false,
        "Compute one inverse kinematics solution for Franka "
        "robot.\n\n"
        "Args:\n"
        "    O_T_EE (Eigen::Matrix<double, 4, 4>): Desired end-effector pose.\n"
        "    q_actual (Vector7d, optional): Current joint angles. "
        "Defaults to kQDefault.\n"
        "    q7 (double, optional): Joint 7 angle. Defaults to M_PI_4.\n"
        "    is_fr3 (bool, optional): Whether to use FR3 joint limits. "
        "Defaults to false.\n\n"
        "Returns:\n"
        "    Vector7d: One IK solution. NaN if no solution.");
  m.def("ik_sample_q7", &frankik::ik_sample_q7, py::arg("O_T_EE"),
        py::arg("q_actual") = std::nullopt, py::arg("is_fr3") = false,
        py::arg("sample_size") = 60, py::arg("sample_interval") = 40,
        py::arg("full_ik") = false,
        "Compute one inverse kinematics solution for Franka with sampling of "
        "joint q7.\n\n"
        "Args:\n"
        "    O_T_EE (np.array): Desired end-effector pose. Shape "
        "(4, 4).\n"
        "    q_actual (np.array, optional): Current joint angles. Shape "
        "(7,).\n"
        "\n"
        "    is_fr3 (bool, optional): Whether to use FR3 joint limits. "
        "Defaults to False.\n"
        "    sample_size (int, optional): How many sample to try for q7. "
        "Defaults to 20.\n"
        "    sample_interval (int, optional): Sample interval for q7 in "
        "degree. Defaults to 90.\n"
        "    full_ik (bool, optional): Whether to use full IK. Defaults to "
        "False.\n"
        "degree. Defaults to False.\n\n"
        "Returns:\n"
        "    list[np.array]: One IK solution. Empty if no solution was found. "
        "Array shape (7,).");
  m.def("fk", &frankik::fk, py::arg("q"),
        "Compute forward kinematics for Franka robot.\n\n"
        "Args:\n"
        "    q (Vector7d): Joint angles.\n\n"
        "Returns:\n"
        "    Eigen::Matrix<double, 4, 4>: End-effector pose.");
}
