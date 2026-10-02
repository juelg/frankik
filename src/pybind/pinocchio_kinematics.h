#ifndef FRANKIK_PINOCCHIO_KINEMATICS_H
#define FRANKIK_PINOCCHIO_KINEMATICS_H

#include <Eigen/Eigen>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <optional>
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

/// Robot description file formats understood by the numerical solver.
enum class ModelFormat { AUTO, MJCF, URDF };

/// Tuning knobs of the closed-loop inverse kinematics (CLIK) iteration.
/// The defaults are identical to the Pinocchio based solver used in the
/// Robot Control Stack (RCS).
struct ClikParameters {
  /// Convergence threshold on the norm of the SE(3) log error (m / rad).
  double eps = 1e-4;
  /// Maximum number of Newton iterations before giving up.
  int max_iterations = 1000;
  /// Integration step size of the joint velocity update.
  double dt = 1e-1;
  /// Levenberg-Marquardt damping added to J J^T before inversion.
  double damping = 1e-6;
  /// Clamp the controlled joints to their position limits after every
  /// iteration. Disabled by default to mirror the RCS behaviour.
  bool clamp_joint_limits = false;
};

/// Generic numerical forward/inverse kinematics backed by Pinocchio.
///
/// The robot is described by a MuJoCo MJCF (or URDF) file. The end-effector
/// is given by the name of a frame in that model (an MJCF `site`, a body or a
/// joint). Poses are expressed relative to an optional `base_frame` (default:
/// the model's world frame).
///
/// Only the first `dof` configuration variables are treated as controllable;
/// all other joints (e.g. gripper fingers) are held fixed. Only models with
/// one-dimensional joints (hinge/slide) are supported, i.e. nq == nv.
class PinocchioKinematics {
 public:
  PinocchioKinematics(const std::string& path, const std::string& tcp_frame,
                      std::optional<std::string> base_frame = std::nullopt,
                      std::optional<int> dof = std::nullopt,
                      ModelFormat format = ModelFormat::AUTO,
                      ClikParameters parameters = ClikParameters())
      : path_(path),
        tcp_frame_(tcp_frame),
        base_frame_(std::move(base_frame)),
        parameters_(parameters) {
    if (!std::ifstream(path).good()) {
      throw std::invalid_argument("Robot model file does not exist: " + path);
    }
    if (format == ModelFormat::AUTO) {
      format = has_urdf_extension(path) ? ModelFormat::URDF : ModelFormat::MJCF;
    }
    if (format == ModelFormat::URDF) {
      pinocchio::urdf::buildModel(path, model_);
    } else {
      pinocchio::mjcf::buildModel(path, model_);
    }
    if (model_.nq != model_.nv) {
      throw std::invalid_argument(
          "frankik only supports robot models with one dimensional joints "
          "(hinge/slide), but the model has nq=" +
          std::to_string(model_.nq) + " and nv=" + std::to_string(model_.nv));
    }
    data_ = pinocchio::Data(model_);

    tcp_frame_id_ = frame_id_or_throw(tcp_frame, "tcp_frame");
    if (base_frame_.has_value()) {
      base_frame_id_ = frame_id_or_throw(*base_frame_, "base_frame");
    }

    dof_ = dof.value_or(model_.nq);
    if (dof_ < 1 || dof_ > model_.nq) {
      throw std::invalid_argument("dof must be in [1, " +
                                  std::to_string(model_.nq) + "], got " +
                                  std::to_string(dof_));
    }
    q_rest_ = pinocchio::neutral(model_);
  }

  /// Forward kinematics: pose of the tcp frame (times `tcp_offset`) relative
  /// to the base frame. `q` has either `dof` or `nq` entries.
  Eigen::Matrix4d forward(
      const Eigen::VectorXd& q,
      const std::optional<Eigen::Matrix4d>& tcp_offset = std::nullopt) {
    const Eigen::VectorXd q_full = full_configuration(q);
    pinocchio::framesForwardKinematics(model_, data_, q_full);
    const pinocchio::SE3 bMt =
        base_placement().actInv(data_.oMf[tcp_frame_id_]);
    if (tcp_offset.has_value()) {
      return (bMt * to_se3(*tcp_offset)).toHomogeneousMatrix();
    }
    return bMt.toHomogeneousMatrix();
  }

  /// Inverse kinematics via damped least squares CLIK (Pinocchio example
  /// algorithm). `pose` is the desired pose of tcp frame * tcp_offset relative
  /// to the base frame, `q0` the initial guess (`dof` or `nq` entries).
  /// Returns the first `dof` joint values or nullopt if not converged.
  std::optional<Eigen::VectorXd> inverse(
      const Eigen::Matrix4d& pose, const Eigen::VectorXd& q0,
      const std::optional<Eigen::Matrix4d>& tcp_offset = std::nullopt) {
    Eigen::VectorXd q = full_configuration(q0);
    pinocchio::SE3 bMdes = to_se3(pose);
    if (tcp_offset.has_value()) {
      bMdes = bMdes * to_se3(*tcp_offset).inverse();
    }

    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();
    Eigen::Matrix<double, 6, 1> err;
    Eigen::VectorXd v(model_.nv);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Data::Matrix6 JJt;
    const int n_uncontrolled = model_.nv - dof_;

    bool success = false;
    for (int i = 0;; i++) {
      pinocchio::forwardKinematics(model_, data_, q);
      pinocchio::updateFramePlacements(model_, data_);
      const pinocchio::SE3 oMdes = base_placement() * bMdes;
      const pinocchio::SE3 iMd = data_.oMf[tcp_frame_id_].actInv(oMdes);
      err = pinocchio::log6(iMd).toVector();
      if (err.norm() < parameters_.eps) {
        success = true;
        break;
      }
      if (i >= parameters_.max_iterations) {
        break;
      }
      pinocchio::computeFrameJacobian(model_, data_, q, tcp_frame_id_, J);
      pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;
      if (n_uncontrolled > 0) {
        // uncontrolled joints must not move
        J.rightCols(n_uncontrolled).setZero();
      }
      JJt.noalias() = J * J.transpose();
      JJt.diagonal().array() += parameters_.damping;
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
      q = pinocchio::integrate(model_, q, v * parameters_.dt);
      if (parameters_.clamp_joint_limits) {
        q.head(dof_) = q.head(dof_)
                           .cwiseMax(model_.lowerPositionLimit.head(dof_))
                           .cwiseMin(model_.upperPositionLimit.head(dof_));
      }
    }
    if (!success) {
      return std::nullopt;
    }
    return Eigen::VectorXd(q.head(dof_));
  }

  // --- model information -------------------------------------------------
  int dof() const { return dof_; }
  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }
  const std::string& path() const { return path_; }
  const std::string& tcp_frame() const { return tcp_frame_; }
  const std::optional<std::string>& base_frame() const { return base_frame_; }
  const std::string& model_name() const { return model_.name; }

  /// Lower position limits of the controlled joints.
  Eigen::VectorXd q_min() const { return model_.lowerPositionLimit.head(dof_); }
  /// Upper position limits of the controlled joints.
  Eigen::VectorXd q_max() const { return model_.upperPositionLimit.head(dof_); }
  /// Neutral configuration of the controlled joints.
  Eigen::VectorXd q_neutral() const {
    return Eigen::VectorXd(pinocchio::neutral(model_).head(dof_));
  }

  /// Values used for the uncontrolled configuration variables when a `q` with
  /// only `dof` entries is given (defaults to the neutral configuration).
  const Eigen::VectorXd& q_rest() const { return q_rest_; }
  void set_q_rest(const Eigen::VectorXd& q_rest) {
    if (q_rest.size() != model_.nq) {
      throw std::invalid_argument(
          "q_rest must have nq=" + std::to_string(model_.nq) +
          " entries, got " + std::to_string(q_rest.size()));
    }
    q_rest_ = q_rest;
  }

  /// Reference configurations, e.g. MJCF `<keyframe>` entries such as "home".
  /// Only the first `dof` entries of each configuration are returned.
  std::map<std::string, Eigen::VectorXd> reference_configurations() const {
    std::map<std::string, Eigen::VectorXd> out;
    for (const auto& [name, q] : model_.referenceConfigurations) {
      out[name] = q.head(dof_);
    }
    return out;
  }

  /// Names of the joints (excluding the "universe" root), in configuration
  /// order.
  std::vector<std::string> joint_names() const {
    std::vector<std::string> names;
    for (pinocchio::JointIndex i = 1; i < model_.joints.size(); ++i) {
      names.push_back(model_.names[i]);
    }
    return names;
  }

  /// Names of all frames (bodies, joints, sites) of the model.
  std::vector<std::string> frame_names() const {
    std::vector<std::string> names;
    for (const auto& frame : model_.frames) {
      names.push_back(frame.name);
    }
    return names;
  }

  ClikParameters& parameters() { return parameters_; }
  void set_parameters(const ClikParameters& parameters) {
    parameters_ = parameters;
  }

 private:
  static bool has_urdf_extension(const std::string& path) {
    const std::string suffix = ".urdf";
    if (path.size() < suffix.size()) {
      return false;
    }
    std::string ext = path.substr(path.size() - suffix.size());
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return ext == suffix;
  }

  static pinocchio::SE3 to_se3(const Eigen::Matrix4d& T) {
    return pinocchio::SE3(T.topLeftCorner<3, 3>(), T.topRightCorner<3, 1>());
  }

  pinocchio::FrameIndex frame_id_or_throw(const std::string& name,
                                          const std::string& what) const {
    if (!model_.existFrame(name)) {
      std::ostringstream msg;
      msg << what << " '" << name << "' not found in model '" << model_.name
          << "' (" << path_ << "). Available frames:";
      for (const auto& frame : model_.frames) {
        msg << " " << frame.name;
      }
      throw std::invalid_argument(msg.str());
    }
    return model_.getFrameId(name);
  }

  /// Expand a `dof` sized vector to the full configuration. Must be called
  /// before accessing data_ dependent quantities.
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

  /// Placement of the base frame in the world frame. Frame placements have to
  /// be up to date.
  pinocchio::SE3 base_placement() const {
    if (!base_frame_id_.has_value()) {
      return pinocchio::SE3::Identity();
    }
    return data_.oMf[*base_frame_id_];
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
