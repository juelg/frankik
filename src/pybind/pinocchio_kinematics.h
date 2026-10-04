#ifndef FRANKIK_PINOCCHIO_KINEMATICS_H
#define FRANKIK_PINOCCHIO_KINEMATICS_H

#include <Eigen/Eigen>
#include <cmath>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace frankik {

enum class ModelFormat { AUTO, MJCF, URDF };

struct ClikParameters {
  double eps = 1e-4;
  int max_iterations = 1000;
  double dt = 0.5;
  double damping = 1e-6;
  bool clamp_joint_limits = true;
};

// Forward kinematics and damped least squares closed-loop inverse kinematics
// (CLIK) for a robot model loaded from an MJCF or URDF file. Poses are
// expressed relative to `base_frame` (world if unset) and refer to
// `tcp_frame`. Only the first `dof` configuration variables are controlled.
class PinocchioKinematics {
 public:
  static constexpr int kGlobalRestarts = 10;

  PinocchioKinematics(const std::string& path, const std::string& tcp_frame,
                      std::optional<std::string> base_frame = std::nullopt,
                      std::optional<int> dof = std::nullopt,
                      ModelFormat format = ModelFormat::AUTO,
                      ClikParameters parameters = ClikParameters())
      : path_(path),
        tcp_frame_(tcp_frame),
        base_frame_(std::move(base_frame)),
        parameters_(parameters) {
    if (format == ModelFormat::AUTO) {
      format = path.ends_with(".urdf") ? ModelFormat::URDF : ModelFormat::MJCF;
    }
    if (format == ModelFormat::URDF) {
      pinocchio::urdf::buildModel(path, model_);
    } else {
      pinocchio::mjcf::buildModel(path, model_);
    }
    if (model_.nq != model_.nv) {
      throw std::invalid_argument(
          "Only models with one dimensional joints are supported, got nq=" +
          std::to_string(model_.nq) + " nv=" + std::to_string(model_.nv));
    }
    data_ = pinocchio::Data(model_);
    tcp_frame_id_ = frame_id(tcp_frame, "tcp_frame");
    if (base_frame_) {
      base_frame_id_ = frame_id(*base_frame_, "base_frame");
    }
    dof_ = dof.value_or(model_.nq);
    if (dof_ < 1 || dof_ > model_.nq) {
      throw std::invalid_argument("dof must be in [1, " +
                                  std::to_string(model_.nq) + "], got " +
                                  std::to_string(dof_));
    }
    q_rest_ = pinocchio::neutral(model_);
  }

  Eigen::Matrix4d forward(
      const Eigen::VectorXd& q,
      const std::optional<Eigen::Matrix4d>& tcp_offset = std::nullopt) {
    pinocchio::framesForwardKinematics(model_, data_, full_configuration(q));
    pinocchio::SE3 pose = base_placement().actInv(data_.oMf[tcp_frame_id_]);
    if (tcp_offset) {
      pose = pose * to_se3(*tcp_offset);
    }
    return pose.toHomogeneousMatrix();
  }

  // Solves for the configuration closest to q0. With global_solution the
  // search restarts from random configurations within the joint limits when
  // the local solve fails, which may land in another arm configuration.
  std::optional<Eigen::VectorXd> inverse(
      const Eigen::Matrix4d& pose, const Eigen::VectorXd& q0,
      const std::optional<Eigen::Matrix4d>& tcp_offset = std::nullopt,
      bool global_solution = false) {
    pinocchio::SE3 target = to_se3(pose);
    if (tcp_offset) {
      target = target * to_se3(*tcp_offset).inverse();
    }
    Eigen::VectorXd q = full_configuration(q0);
    if (const auto solution = solve(target, q)) {
      return solution;
    }
    if (!global_solution) {
      return std::nullopt;
    }
    std::mt19937 rng(0);
    for (int attempt = 0; attempt < kGlobalRestarts; ++attempt) {
      q.head(dof_) = random_configuration(q.head(dof_), rng);
      if (const auto solution = solve(target, q)) {
        return solution;
      }
    }
    return std::nullopt;
  }

  int dof() const { return dof_; }
  int nq() const { return model_.nq; }
  const std::string& path() const { return path_; }
  const std::string& tcp_frame() const { return tcp_frame_; }
  const std::optional<std::string>& base_frame() const { return base_frame_; }
  Eigen::VectorXd q_min() const { return model_.lowerPositionLimit.head(dof_); }
  Eigen::VectorXd q_max() const { return model_.upperPositionLimit.head(dof_); }
  Eigen::VectorXd q_neutral() const {
    return pinocchio::neutral(model_).head(dof_);
  }
  const Eigen::VectorXd& q_rest() const { return q_rest_; }
  void set_q_rest(const Eigen::VectorXd& q_rest) {
    if (q_rest.size() != model_.nq) {
      throw std::invalid_argument(
          "q_rest must have nq=" + std::to_string(model_.nq) + " entries");
    }
    q_rest_ = q_rest;
  }
  const ClikParameters& parameters() const { return parameters_; }
  void set_parameters(const ClikParameters& parameters) {
    parameters_ = parameters;
  }

  std::map<std::string, Eigen::VectorXd> reference_configurations() const {
    std::map<std::string, Eigen::VectorXd> configurations;
    for (const auto& [name, q] : model_.referenceConfigurations) {
      configurations[name] = q.head(dof_);
    }
    return configurations;
  }

  std::vector<std::string> joint_names() const {
    return {model_.names.begin() + 1, model_.names.end()};
  }

  std::vector<std::string> frame_names() const {
    std::vector<std::string> names;
    for (const auto& frame : model_.frames) {
      names.push_back(frame.name);
    }
    return names;
  }

 private:
  // Damped least squares closed-loop IK as in the Pinocchio examples: each
  // iteration integrates dt times the Gauss-Newton step with a constant
  // damping, which converges linearly (dt = 0.1 needs ~100 iterations, 0.5
  // about 5x fewer at the same success rate). A Levenberg-Marquardt variant
  // with a full step and error proportional damping (Wn = lambda * ||err||^2,
  // Chan & Lawrence / Sugihara) would converge quadratically near the solution
  // and is the next step if the solver needs to get faster.
  std::optional<Eigen::VectorXd> solve(const pinocchio::SE3& target,
                                       Eigen::VectorXd q) {
    const auto q_min = model_.lowerPositionLimit.head(dof_);
    const auto q_max = model_.upperPositionLimit.head(dof_);
    pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, model_.nv);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Data::Matrix6 JJt;
    Eigen::VectorXd v(model_.nv);
    for (int i = 0; i <= parameters_.max_iterations; ++i) {
      pinocchio::forwardKinematics(model_, data_, q);
      pinocchio::updateFramePlacements(model_, data_);
      const pinocchio::SE3 iMd =
          data_.oMf[tcp_frame_id_].actInv(base_placement() * target);
      const pinocchio::Motion::Vector6 err = pinocchio::log6(iMd).toVector();
      if (err.norm() < parameters_.eps) {
        return Eigen::VectorXd(q.head(dof_));
      }
      pinocchio::computeFrameJacobian(model_, data_, q, tcp_frame_id_, J);
      pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;
      J.rightCols(model_.nv - dof_).setZero();
      JJt.noalias() = J * J.transpose();
      JJt.diagonal().array() += parameters_.damping;
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
      q = pinocchio::integrate(model_, q, v * parameters_.dt);
      if (parameters_.clamp_joint_limits) {
        q.head(dof_) = q.head(dof_).cwiseMax(q_min).cwiseMin(q_max);
      }
    }
    return std::nullopt;
  }

  // Uniform sample within the joint limits, within +-pi around q for
  // unlimited joints.
  Eigen::VectorXd random_configuration(const Eigen::VectorXd& q,
                                       std::mt19937& rng) const {
    Eigen::VectorXd sample(dof_);
    for (int j = 0; j < dof_; ++j) {
      double lower = model_.lowerPositionLimit[j];
      double upper = model_.upperPositionLimit[j];
      if (!std::isfinite(lower) || !std::isfinite(upper)) {
        lower = q[j] - M_PI;
        upper = q[j] + M_PI;
      }
      sample[j] = std::uniform_real_distribution<double>(lower, upper)(rng);
    }
    return sample;
  }

  static pinocchio::SE3 to_se3(const Eigen::Matrix4d& T) {
    return {T.topLeftCorner<3, 3>(), T.topRightCorner<3, 1>()};
  }

  pinocchio::FrameIndex frame_id(const std::string& name,
                                 const std::string& argument) const {
    if (!model_.existFrame(name)) {
      std::ostringstream msg;
      msg << argument << " '" << name << "' not found in " << path_
          << ". Available frames:";
      for (const auto& frame : model_.frames) {
        msg << " " << frame.name;
      }
      throw std::invalid_argument(msg.str());
    }
    return model_.getFrameId(name);
  }

  Eigen::VectorXd full_configuration(const Eigen::VectorXd& q) const {
    if (q.size() == model_.nq) {
      return q;
    }
    if (q.size() != dof_) {
      throw std::invalid_argument("q must have dof=" + std::to_string(dof_) +
                                  " or nq=" + std::to_string(model_.nq) +
                                  " entries, got " + std::to_string(q.size()));
    }
    Eigen::VectorXd q_full = q_rest_;
    q_full.head(dof_) = q;
    return q_full;
  }

  pinocchio::SE3 base_placement() const {
    return base_frame_id_ ? data_.oMf[*base_frame_id_]
                          : pinocchio::SE3::Identity();
  }

  std::string path_;
  std::string tcp_frame_;
  std::optional<std::string> base_frame_;
  ClikParameters parameters_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex tcp_frame_id_ = 0;
  std::optional<pinocchio::FrameIndex> base_frame_id_;
  int dof_ = 0;
  Eigen::VectorXd q_rest_;
};

}  // namespace frankik

#endif  // FRANKIK_PINOCCHIO_KINEMATICS_H
