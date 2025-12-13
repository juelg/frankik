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

  m.def("ik_full", &frankik::ik_full, py::arg("O_T_EE"),
        py::arg("q_actual_array") = frankik::kQDefault, py::arg("q7") = M_PI_4, py::arg("is_fr3") = false,
        "Compute full inverse kinematics for Franka Emika Panda robot.\n\n"
        "Args:\n"
        "    O_T_EE (Eigen::Matrix<double, 4, 4>): Desired end-effector pose.\n"
        "    q_actual_array (Vector7d, optional): Current joint angles. "
        "Defaults to kQDefault.\n"
        "    q7 (double, optional): Joint 7 angle. Defaults to M_PI_4.\n"
        "    is_fr3 (bool, optional): Whether to use FR3 joint limits. Defaults to false.\n\n"
        "Returns:\n"
        "    Eigen::Matrix<double, 4, 7>: All possible IK solutions (up to 4). "
        "NaN if no solution.");
  m.def("ik", &frankik::ik, py::arg("O_T_EE"),
        py::arg("q_actual_array") = frankik::kQDefault, py::arg("q7") = M_PI_4, py::arg("is_fr3") = false,
        "Compute one inverse kinematics solution for Franka Emika Panda "
        "robot.\n\n"
        "Args:\n"
        "    O_T_EE (Eigen::Matrix<double, 4, 4>): Desired end-effector pose.\n"
        "    q_actual_array (Vector7d, optional): Current joint angles. "
        "Defaults to kQDefault.\n"
        "    q7 (double, optional): Joint 7 angle. Defaults to M_PI_4.\n"
        "    is_fr3 (bool, optional): Whether to use FR3 joint limits. Defaults to false.\n\n"
        "Returns:\n"
        "    Vector7d: One IK solution. NaN if no solution.");
  m.def("fk", &frankik::fk, py::arg("q"),
        "Compute forward kinematics for Franka Emika Panda robot.\n\n"
        "Args:\n"
        "    q (Vector7d): Joint angles.\n\n"
        "Returns:\n"
        "    Eigen::Matrix<double, 4, 4>: End-effector pose.");
}
