#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pinocchio/config.hpp"
#include "pinocchio_kinematics.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using frankik::ClikParameters;
using frankik::ModelFormat;
using frankik::PinocchioKinematics;

PYBIND11_MODULE(_pin, m) {
  m.doc() = "Pinocchio based numerical kinematics of frankik";
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
  m.attr("pinocchio_version") = PINOCCHIO_VERSION;

  py::enum_<ModelFormat>(m, "ModelFormat")
      .value("AUTO", ModelFormat::AUTO, "By file extension: .urdf or MJCF")
      .value("MJCF", ModelFormat::MJCF)
      .value("URDF", ModelFormat::URDF);

  py::class_<ClikParameters>(m, "ClikParameters",
                             "Parameters of the closed-loop IK iteration.")
      .def(py::init([](double eps, int max_iterations, double dt,
                       double damping, bool clamp_joint_limits) {
             return ClikParameters{eps, max_iterations, dt, damping,
                                   clamp_joint_limits};
           }),
           py::arg("eps") = ClikParameters().eps,
           py::arg("max_iterations") = ClikParameters().max_iterations,
           py::arg("dt") = ClikParameters().dt,
           py::arg("damping") = ClikParameters().damping,
           py::arg("clamp_joint_limits") = ClikParameters().clamp_joint_limits)
      .def_readwrite("eps", &ClikParameters::eps,
                     "Convergence threshold on the SE(3) log error norm")
      .def_readwrite("max_iterations", &ClikParameters::max_iterations)
      .def_readwrite("dt", &ClikParameters::dt,
                     "Integration step of the velocity update")
      .def_readwrite("damping", &ClikParameters::damping,
                     "Levenberg-Marquardt damping")
      .def_readwrite("clamp_joint_limits", &ClikParameters::clamp_joint_limits,
                     "Clamp the controlled joints to their limits after every "
                     "iteration")
      .def("__repr__", [](const ClikParameters& p) {
        return "ClikParameters(eps=" + std::to_string(p.eps) +
               ", max_iterations=" + std::to_string(p.max_iterations) +
               ", dt=" + std::to_string(p.dt) +
               ", damping=" + std::to_string(p.damping) +
               ", clamp_joint_limits=" +
               (p.clamp_joint_limits ? "True" : "False") + ")";
      });

  py::class_<PinocchioKinematics>(
      m, "PinocchioKinematics",
      "Numerical forward/inverse kinematics of a robot described by an MJCF "
      "or URDF file. Poses are 4x4 matrices of `tcp_frame` (times an optional "
      "tcp offset) relative to `base_frame` (default: world), only the first "
      "`dof` configuration variables are controlled.")
      .def(py::init<const std::string&, const std::string&,
                    std::optional<std::string>, std::optional<int>, ModelFormat,
                    ClikParameters>(),
           py::arg("path"), py::arg("tcp_frame"),
           py::arg("base_frame") = std::nullopt, py::arg("dof") = std::nullopt,
           py::arg("format") = ModelFormat::AUTO,
           py::arg("parameters") = ClikParameters())
      .def("forward", &PinocchioKinematics::forward, py::arg("q"),
           py::arg("tcp_offset") = std::nullopt,
           "Pose of tcp_frame * tcp_offset in base_frame for q (dof or nq "
           "entries).")
      .def("inverse", &PinocchioKinematics::inverse, py::arg("pose"),
           py::arg("q0"), py::arg("tcp_offset") = std::nullopt,
           "Controlled joint values reaching pose from seed q0 (dof or nq "
           "entries), None if the solver did not converge.")
      .def_property_readonly("dof", &PinocchioKinematics::dof)
      .def_property_readonly("nq", &PinocchioKinematics::nq)
      .def_property_readonly("path", &PinocchioKinematics::path)
      .def_property_readonly("tcp_frame", &PinocchioKinematics::tcp_frame)
      .def_property_readonly("base_frame", &PinocchioKinematics::base_frame)
      .def_property_readonly("q_min", &PinocchioKinematics::q_min)
      .def_property_readonly("q_max", &PinocchioKinematics::q_max)
      .def_property_readonly("q_neutral", &PinocchioKinematics::q_neutral)
      .def_property("q_rest", &PinocchioKinematics::q_rest,
                    &PinocchioKinematics::set_q_rest,
                    "Full configuration (nq) used for the uncontrolled joints")
      .def_property("parameters", &PinocchioKinematics::parameters,
                    &PinocchioKinematics::set_parameters)
      .def("reference_configurations",
           &PinocchioKinematics::reference_configurations,
           "MJCF keyframes restricted to the controlled joints")
      .def("joint_names", &PinocchioKinematics::joint_names)
      .def("frame_names", &PinocchioKinematics::frame_names);
}
