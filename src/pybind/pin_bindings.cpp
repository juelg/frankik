#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pinocchio/config.hpp"
#include "pinocchio_kinematics.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_pin, m) {
  m.doc() =
      "Python bindings for the Pinocchio based numerical kinematics of "
      "frankik";
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
  m.attr("pinocchio_version") = PINOCCHIO_VERSION;

  py::enum_<frankik::ModelFormat>(m, "ModelFormat",
                                  "Robot description file format.")
      .value("AUTO", frankik::ModelFormat::AUTO,
             "Guess from the file extension (`.urdf` -> URDF, otherwise MJCF)")
      .value("MJCF", frankik::ModelFormat::MJCF, "MuJoCo XML")
      .value("URDF", frankik::ModelFormat::URDF, "URDF");

  py::class_<frankik::ClikParameters>(
      m, "ClikParameters",
      "Tuning parameters of the closed-loop inverse kinematics iteration.")
      .def(py::init<>())
      .def(py::init([](double eps, int max_iterations, double dt,
                       double damping, bool clamp_joint_limits) {
             frankik::ClikParameters p;
             p.eps = eps;
             p.max_iterations = max_iterations;
             p.dt = dt;
             p.damping = damping;
             p.clamp_joint_limits = clamp_joint_limits;
             return p;
           }),
           py::arg("eps") = 1e-4, py::arg("max_iterations") = 1000,
           py::arg("dt") = 1e-1, py::arg("damping") = 1e-6,
           py::arg("clamp_joint_limits") = false)
      .def_readwrite("eps", &frankik::ClikParameters::eps,
                     "Convergence threshold on the SE(3) log error norm.")
      .def_readwrite("max_iterations", &frankik::ClikParameters::max_iterations,
                     "Maximum number of iterations.")
      .def_readwrite("dt", &frankik::ClikParameters::dt,
                     "Integration step of the velocity update.")
      .def_readwrite("damping", &frankik::ClikParameters::damping,
                     "Levenberg-Marquardt damping.")
      .def_readwrite("clamp_joint_limits",
                     &frankik::ClikParameters::clamp_joint_limits,
                     "Clamp controlled joints to their limits after every "
                     "iteration.")
      .def("__repr__", [](const frankik::ClikParameters& p) {
        return "ClikParameters(eps=" + std::to_string(p.eps) +
               ", max_iterations=" + std::to_string(p.max_iterations) +
               ", dt=" + std::to_string(p.dt) +
               ", damping=" + std::to_string(p.damping) +
               ", clamp_joint_limits=" +
               (p.clamp_joint_limits ? "True" : "False") + ")";
      });

  py::class_<frankik::PinocchioKinematics>(
      m, "PinocchioKinematics",
      "Numerical forward/inverse kinematics for arbitrary robots described "
      "by a MuJoCo MJCF (or URDF) file, computed with Pinocchio.\n\n"
      "Poses are 4x4 homogeneous matrices of the `tcp_frame` (optionally "
      "times a tcp offset) expressed in `base_frame` (default: world).")
      .def(py::init<const std::string&, const std::string&,
                    std::optional<std::string>, std::optional<int>,
                    frankik::ModelFormat, frankik::ClikParameters>(),
           py::arg("path"), py::arg("tcp_frame"),
           py::arg("base_frame") = std::nullopt, py::arg("dof") = std::nullopt,
           py::arg("format") = frankik::ModelFormat::AUTO,
           py::arg("parameters") = frankik::ClikParameters(),
           "Load a robot model.\n\n"
           "Args:\n"
           "    path (str): Path to the MJCF (or URDF) file.\n"
           "    tcp_frame (str): Name of the end-effector frame (MJCF site, "
           "body or joint).\n"
           "    base_frame (str, optional): Frame poses are expressed in. "
           "Defaults to the world frame.\n"
           "    dof (int, optional): Number of controlled joints (the first "
           "`dof` configuration variables). Defaults to all joints.\n"
           "    format (ModelFormat, optional): File format. Defaults to "
           "AUTO.\n"
           "    parameters (ClikParameters, optional): Solver parameters.")
      .def("forward", &frankik::PinocchioKinematics::forward, py::arg("q"),
           py::arg("tcp_offset") = std::nullopt,
           "Forward kinematics.\n\n"
           "Args:\n"
           "    q (np.ndarray): Joint configuration with `dof` or `nq` "
           "entries.\n"
           "    tcp_offset (np.ndarray, optional): 4x4 offset applied to the "
           "tcp frame.\n\n"
           "Returns:\n"
           "    np.ndarray: 4x4 pose of tcp_frame * tcp_offset in base_frame.")
      .def("inverse", &frankik::PinocchioKinematics::inverse, py::arg("pose"),
           py::arg("q0"), py::arg("tcp_offset") = std::nullopt,
           "Inverse kinematics (damped least squares CLIK).\n\n"
           "Args:\n"
           "    pose (np.ndarray): Desired 4x4 pose of tcp_frame * tcp_offset "
           "in base_frame.\n"
           "    q0 (np.ndarray): Initial guess with `dof` or `nq` entries.\n"
           "    tcp_offset (np.ndarray, optional): 4x4 offset applied to the "
           "tcp frame.\n\n"
           "Returns:\n"
           "    np.ndarray | None: Joint values of the `dof` controlled joints "
           "or None if the solver did not converge.")
      .def_property_readonly("dof", &frankik::PinocchioKinematics::dof,
                             "Number of controlled joints.")
      .def_property_readonly("nq", &frankik::PinocchioKinematics::nq,
                             "Size of the full model configuration.")
      .def_property_readonly("nv", &frankik::PinocchioKinematics::nv,
                             "Size of the full model velocity.")
      .def_property_readonly("path", &frankik::PinocchioKinematics::path,
                             "Path of the loaded robot model.")
      .def_property_readonly("tcp_frame",
                             &frankik::PinocchioKinematics::tcp_frame,
                             "Name of the end-effector frame.")
      .def_property_readonly("base_frame",
                             &frankik::PinocchioKinematics::base_frame,
                             "Name of the base frame or None for world.")
      .def_property_readonly("model_name",
                             &frankik::PinocchioKinematics::model_name,
                             "Name of the model as given in the file.")
      .def_property_readonly("q_min", &frankik::PinocchioKinematics::q_min,
                             "Lower position limits of the controlled joints.")
      .def_property_readonly("q_max", &frankik::PinocchioKinematics::q_max,
                             "Upper position limits of the controlled joints.")
      .def_property_readonly("q_neutral",
                             &frankik::PinocchioKinematics::q_neutral,
                             "Neutral configuration of the controlled joints.")
      .def_property("q_rest", &frankik::PinocchioKinematics::q_rest,
                    &frankik::PinocchioKinematics::set_q_rest,
                    "Full (nq) configuration whose tail is used for the "
                    "uncontrolled joints.")
      .def_property(
          "parameters",
          [](frankik::PinocchioKinematics& self) { return self.parameters(); },
          &frankik::PinocchioKinematics::set_parameters,
          "Solver parameters (ClikParameters).")
      .def("reference_configurations",
           &frankik::PinocchioKinematics::reference_configurations,
           "Named reference configurations (MJCF keyframes), restricted to the "
           "controlled joints.")
      .def("joint_names", &frankik::PinocchioKinematics::joint_names,
           "Joint names in configuration order.")
      .def("frame_names", &frankik::PinocchioKinematics::frame_names,
           "All frame names (bodies, joints, sites) of the model.");
}
