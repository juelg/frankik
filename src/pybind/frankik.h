#ifndef FRANKIK_H
#define FRANKIK_H

#include <array>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <limits>

namespace frankik {

typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;

const double d1 = 0.3330;
const double d3 = 0.3160;
const double d5 = 0.3840;
const double d7e = 0.107;
const double a4 = 0.0825;
const double a7 = 0.0880;

// Pre-calculated squares for IK speed
const double LL24 = 0.10666225;     // a4^2 + d3^2
const double LL46 = 0.15426225;     // a4^2 + d5^2
const double L24 = 0.326591870689;  // sqrt(LL24)
const double L46 = 0.392762332715;  // sqrt(LL46)

// Pre-calculated geometric angles
const double thetaH46 = 1.35916951803;   // atan(d5/a4);
const double theta342 = 1.31542071191;   // atan(d3/a4);
const double theta46H = 0.211626808766;  // acot(d5/a4);

const std::array<double, 7> q_min_panda = {
    {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
const std::array<double, 7> q_max_panda = {
    {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};

const std::array<double, 7> q_min_fr3 = {
    {-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895}};
const std::array<double, 7> q_max_fr3 = {
    {2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895}};

const Vector7d kQDefault =
    (Vector7d() << 0.0, -M_PI_4, 0.0, -3 * M_PI_4, 0.0, M_PI_2, 0.0).finished();

Eigen::Matrix<double, 4, 7> ik_full(
    Eigen::Matrix<double, 4, 4> O_T_EE,
    std::optional<Vector7d> q_actual = std::nullopt, double q7 = M_PI_4,
    bool is_fr3 = false) {
  Vector7d q_actual_array = q_actual.value_or(kQDefault);
  const Eigen::Matrix<double, 4, 7> q_all_NAN =
      Eigen::Matrix<double, 4, 7>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  const Vector7d q_NAN =
      Vector7d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, 4, 7> q_all = q_all_NAN;

  const std::array<double, 7> q_min = is_fr3 ? q_min_fr3 : q_min_panda;
  const std::array<double, 7> q_max = is_fr3 ? q_max_fr3 : q_max_panda;

  if (q7 <= q_min[6] || q7 >= q_max[6]) return q_all_NAN;
  for (int i = 0; i < 4; i++) q_all(i, 6) = q7;

  // compute p_6
  Eigen::Matrix3d R_EE = O_T_EE.topLeftCorner<3, 3>();
  Eigen::Vector3d z_EE = O_T_EE.block<3, 1>(0, 2);
  Eigen::Vector3d p_EE = O_T_EE.block<3, 1>(0, 3);

  // Use constant d7e
  Eigen::Vector3d p_7 = p_EE - d7e * z_EE;

  Eigen::Vector3d x_EE_6;
  x_EE_6 << std::cos(q7), -std::sin(q7), 0.0;
  Eigen::Vector3d x_6 = R_EE * x_EE_6;
  x_6 /= x_6.norm();

  // Use constant a7
  Eigen::Vector3d p_6 = p_7 - a7 * x_6;

  // compute q4
  Eigen::Vector3d p_2;
  p_2 << 0.0, 0.0, d1;  // Use constant d1
  Eigen::Vector3d V26 = p_6 - p_2;

  double LL26 = V26.squaredNorm();
  double L26 = std::sqrt(LL26);

  if (L24 + L46 < L26 || L24 + L26 < L46 || L26 + L46 < L24) return q_all_NAN;

  double theta246 = std::acos((LL24 + LL46 - LL26) / 2.0 / L24 / L46);
  double q4 = theta246 + thetaH46 + theta342 - 2.0 * M_PI;
  if (q4 <= q_min[3] || q4 >= q_max[3]) return q_all_NAN;
  for (int i = 0; i < 4; i++) q_all(i, 3) = q4;

  // compute q6
  double theta462 = std::acos((LL26 + LL46 - LL24) / 2.0 / L26 / L46);
  double theta26H = theta46H + theta462;
  double D26 = -L26 * std::cos(theta26H);

  Eigen::Vector3d Z_6 = z_EE.cross(x_6);
  Eigen::Vector3d Y_6 = Z_6.cross(x_6);
  Eigen::Matrix3d R_6;
  R_6.col(0) = x_6;
  R_6.col(1) = Y_6 / Y_6.norm();
  R_6.col(2) = Z_6 / Z_6.norm();
  Eigen::Vector3d V_6_62 = R_6.transpose() * (-V26);

  double Phi6 = std::atan2(V_6_62[1], V_6_62[0]);
  double Theta6 = std::asin(D26 / std::hypot(V_6_62[0], V_6_62[1]));

  std::array<double, 2> q6;
  q6[0] = M_PI - Theta6 - Phi6;
  q6[1] = Theta6 - Phi6;

  for (int i = 0; i < 2; i++) {
    if (q6[i] <= q_min[5])
      q6[i] += 2.0 * M_PI;
    else if (q6[i] >= q_max[5])
      q6[i] -= 2.0 * M_PI;

    if (q6[i] <= q_min[5] || q6[i] >= q_max[5]) {
      q_all.row(2 * i) = q_NAN;
      q_all.row(2 * i + 1) = q_NAN;
    } else {
      q_all(2 * i, 5) = q6[i];
      q_all(2 * i + 1, 5) = q6[i];
    }
  }
  if (std::isnan(q_all(0, 5)) && std::isnan(q_all(2, 5))) return q_all_NAN;

  // compute q1 & q2
  double thetaP26 = 3.0 * M_PI_2 - theta462 - theta246 - theta342;
  double thetaP = M_PI - thetaP26 - theta26H;
  double LP6 = L26 * sin(thetaP26) / std::sin(thetaP);

  std::array<Eigen::Vector3d, 4> z_5_all;
  std::array<Eigen::Vector3d, 4> V2P_all;

  for (int i = 0; i < 2; i++) {
    Eigen::Vector3d z_6_5;
    z_6_5 << std::sin(q6[i]), std::cos(q6[i]), 0.0;
    Eigen::Vector3d z_5 = R_6 * z_6_5;
    Eigen::Vector3d V2P = p_6 - LP6 * z_5 - p_2;

    z_5_all[2 * i] = z_5;
    z_5_all[2 * i + 1] = z_5;
    V2P_all[2 * i] = V2P;
    V2P_all[2 * i + 1] = V2P;

    double L2P = V2P.norm();

    if (std::fabs(V2P[2] / L2P) > 0.99999) {
      q_all(2 * i, 0) = q_actual_array[0];
      q_all(2 * i, 1) = 0.0;
      q_all(2 * i + 1, 0) = q_actual_array[0];
      q_all(2 * i + 1, 1) = 0.0;
    } else {
      q_all(2 * i, 0) = std::atan2(V2P[1], V2P[0]);
      q_all(2 * i, 1) = std::acos(V2P[2] / L2P);
      if (q_all(2 * i, 0) < 0)
        q_all(2 * i + 1, 0) = q_all(2 * i, 0) + M_PI;
      else
        q_all(2 * i + 1, 0) = q_all(2 * i, 0) - M_PI;
      q_all(2 * i + 1, 1) = -q_all(2 * i, 1);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (q_all(i, 0) <= q_min[0] || q_all(i, 0) >= q_max[0] ||
        q_all(i, 1) <= q_min[1] || q_all(i, 1) >= q_max[1]) {
      q_all.row(i) = q_NAN;
      continue;
    }

    // compute q3
    Eigen::Vector3d z_3 = V2P_all[i] / V2P_all[i].norm();
    Eigen::Vector3d Y_3 = -V26.cross(V2P_all[i]);
    Eigen::Vector3d y_3 = Y_3 / Y_3.norm();
    Eigen::Vector3d x_3 = y_3.cross(z_3);

    // Explicit Rotation Matrix Construction
    double c1 = std::cos(q_all(i, 0));
    double s1 = std::sin(q_all(i, 0));
    double c2 = std::cos(q_all(i, 1));
    double s2 = std::sin(q_all(i, 1));

    Eigen::Matrix3d R_2;
    R_2 << c1 * c2, -c1 * s2, -s1, s1 * c2, -s1 * s2, c1, -s2, -c2, 0;

    Eigen::Vector3d x_2_3 = R_2.transpose() * x_3;
    q_all(i, 2) = std::atan2(x_2_3[2], x_2_3[0]);

    if (q_all(i, 2) <= q_min[2] || q_all(i, 2) >= q_max[2]) {
      q_all.row(i) = q_NAN;
      continue;
    }

    // compute q5
    // Use constants d3, a4, d5
    Eigen::Vector3d VH4 = p_2 + d3 * z_3 + a4 * x_3 - p_6 + d5 * z_5_all[i];

    double c6 = std::cos(q_all(i, 5));
    double s6 = std::sin(q_all(i, 5));
    Eigen::Matrix3d R_5_6;
    R_5_6 << c6, -s6, 0.0, 0.0, 0.0, -1.0, s6, c6, 0.0;

    Eigen::Matrix3d R_5 = R_6 * R_5_6.transpose();
    Eigen::Vector3d V_5_H4 = R_5.transpose() * VH4;

    q_all(i, 4) = -std::atan2(V_5_H4[1], V_5_H4[0]);
    if (q_all(i, 4) <= q_min[4] || q_all(i, 4) >= q_max[4]) {
      q_all.row(i) = q_NAN;
      continue;
    }
  }

  return q_all;
}

Vector7d ik(Eigen::Matrix<double, 4, 4> O_T_EE,
            std::optional<Vector7d> q_actual = std::nullopt, double q7 = M_PI_4,
            bool is_fr3 = false) {
  Vector7d q_actual_array = q_actual.value_or(kQDefault);
  const Vector7d q_NAN =
      Vector7d::Constant(std::numeric_limits<double>::quiet_NaN());
  Vector7d q;

  const std::array<double, 7> q_min = is_fr3 ? q_min_fr3 : q_min_panda;
  const std::array<double, 7> q_max = is_fr3 ? q_max_fr3 : q_max_panda;

  // return NAN if input q7 is out of range
  if (q7 <= q_min[6] || q7 >= q_max[6])
    return q_NAN;
  else
    q[6] = q7;

  // FK for getting current case id
  double c1_a = std::cos(q_actual_array[0]);
  double s1_a = std::sin(q_actual_array[0]);
  double c2_a = std::cos(q_actual_array[1]);
  double s2_a = std::sin(q_actual_array[1]);
  double c3_a = std::cos(q_actual_array[2]);
  double s3_a = std::sin(q_actual_array[2]);
  double c4_a = std::cos(q_actual_array[3]);
  double s4_a = std::sin(q_actual_array[3]);
  double c5_a = std::cos(q_actual_array[4]);
  double s5_a = std::sin(q_actual_array[4]);
  double c6_a = std::cos(q_actual_array[5]);
  double s6_a = std::sin(q_actual_array[5]);

  std::array<Eigen::Matrix<double, 4, 4>, 7> As_a;
  As_a[0] << c1_a, -s1_a, 0.0, 0.0,  // O1
      s1_a, c1_a, 0.0, 0.0, 0.0, 0.0, 1.0, d1, 0.0, 0.0, 0.0, 1.0;
  As_a[1] << c2_a, -s2_a, 0.0, 0.0,  // O2
      0.0, 0.0, 1.0, 0.0, -s2_a, -c2_a, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  As_a[2] << c3_a, -s3_a, 0.0, 0.0,  // O3
      0.0, 0.0, -1.0, -d3, s3_a, c3_a, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  As_a[3] << c4_a, -s4_a, 0.0, a4,  // O4
      0.0, 0.0, -1.0, 0.0, s4_a, c4_a, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  As_a[4] << 1.0, 0.0, 0.0, -a4,  // H
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  As_a[5] << c5_a, -s5_a, 0.0, 0.0,  // O5
      0.0, 0.0, 1.0, d5, -s5_a, -c5_a, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  As_a[6] << c6_a, -s6_a, 0.0, 0.0,  // O6
      0.0, 0.0, -1.0, 0.0, s6_a, c6_a, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  std::array<Eigen::Matrix<double, 4, 4>, 7> Ts_a;
  Ts_a[0] = As_a[0];
  for (unsigned int j = 1; j < 7; j++) Ts_a[j] = Ts_a[j - 1] * As_a[j];

  // identify q6 case
  Eigen::Vector3d V62_a = Ts_a[1].block<3, 1>(0, 3) - Ts_a[6].block<3, 1>(0, 3);
  Eigen::Vector3d V6H_a = Ts_a[4].block<3, 1>(0, 3) - Ts_a[6].block<3, 1>(0, 3);
  Eigen::Vector3d Z6_a = Ts_a[6].block<3, 1>(0, 2);
  bool is_case6_0 = ((V6H_a.cross(V62_a)).transpose() * Z6_a <= 0);

  // identify q1 case
  bool is_case1_1 = (q_actual_array[1] < 0);

  // IK: compute p_6
  Eigen::Matrix3d R_EE = O_T_EE.topLeftCorner<3, 3>();
  Eigen::Vector3d z_EE = O_T_EE.block<3, 1>(0, 2);
  Eigen::Vector3d p_EE = O_T_EE.block<3, 1>(0, 3);

  Eigen::Vector3d p_7 = p_EE - d7e * z_EE;

  Eigen::Vector3d x_EE_6;
  x_EE_6 << std::cos(q7), -std::sin(q7), 0.0;
  Eigen::Vector3d x_6 = R_EE * x_EE_6;
  x_6 /= x_6.norm();  // visibly increases accuracy

  Eigen::Vector3d p_6 = p_7 - a7 * x_6;

  // IK: compute q4
  Eigen::Vector3d p_2;
  p_2 << 0.0, 0.0, d1;
  Eigen::Vector3d V26 = p_6 - p_2;

  double LL26 = V26.squaredNorm();  // Optimized to squaredNorm()
  double L26 = std::sqrt(LL26);

  if (L24 + L46 < L26 || L24 + L26 < L46 || L26 + L46 < L24) return q_NAN;

  double theta246 = std::acos((LL24 + LL46 - LL26) / 2.0 / L24 / L46);
  q[3] = theta246 + thetaH46 + theta342 - 2.0 * M_PI;
  if (q[3] <= q_min[3] || q[3] >= q_max[3]) return q_NAN;

  // IK: compute q6
  double theta462 = std::acos((LL26 + LL46 - LL24) / 2.0 / L26 / L46);
  double theta26H = theta46H + theta462;
  double D26 = -L26 * std::cos(theta26H);

  Eigen::Vector3d Z_6 = z_EE.cross(x_6);
  Eigen::Vector3d Y_6 = Z_6.cross(x_6);
  Eigen::Matrix3d R_6;
  R_6.col(0) = x_6;
  R_6.col(1) = Y_6 / Y_6.norm();
  R_6.col(2) = Z_6 / Z_6.norm();
  Eigen::Vector3d V_6_62 = R_6.transpose() * (-V26);

  double Phi6 = std::atan2(V_6_62[1], V_6_62[0]);
  double Theta6 =
      std::asin(D26 / std::sqrt(V_6_62[0] * V_6_62[0] + V_6_62[1] * V_6_62[1]));

  if (is_case6_0)
    q[5] = M_PI - Theta6 - Phi6;
  else
    q[5] = Theta6 - Phi6;

  if (q[5] <= q_min[5])
    q[5] += 2.0 * M_PI;
  else if (q[5] >= q_max[5])
    q[5] -= 2.0 * M_PI;

  if (q[5] <= q_min[5] || q[5] >= q_max[5]) return q_NAN;

  // IK: compute q1 & q2
  double thetaP26 = 3.0 * M_PI_2 - theta462 - theta246 - theta342;
  double thetaP = M_PI - thetaP26 - theta26H;
  double LP6 = L26 * sin(thetaP26) / std::sin(thetaP);

  Eigen::Vector3d z_6_5;
  z_6_5 << std::sin(q[5]), std::cos(q[5]), 0.0;
  Eigen::Vector3d z_5 = R_6 * z_6_5;
  Eigen::Vector3d V2P = p_6 - LP6 * z_5 - p_2;

  double L2P = V2P.norm();

  if (std::fabs(V2P[2] / L2P) > 0.99999) {
    q[0] = q_actual_array[0];
    q[1] = 0.0;
  } else {
    q[0] = std::atan2(V2P[1], V2P[0]);
    q[1] = std::acos(V2P[2] / L2P);
    if (is_case1_1) {
      if (q[0] < 0.0)
        q[0] += M_PI;
      else
        q[0] -= M_PI;
      q[1] = -q[1];
    }
  }

  if (q[0] <= q_min[0] || q[0] >= q_max[0] || q[1] <= q_min[1] ||
      q[1] >= q_max[1])
    return q_NAN;

  // IK: compute q3
  Eigen::Vector3d z_3 = V2P / V2P.norm();
  Eigen::Vector3d Y_3 = -V26.cross(V2P);
  Eigen::Vector3d y_3 = Y_3 / Y_3.norm();
  Eigen::Vector3d x_3 = y_3.cross(z_3);
  Eigen::Matrix3d R_1;
  double c1 = std::cos(q[0]);
  double s1 = std::sin(q[0]);
  R_1 << c1, -s1, 0.0, s1, c1, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix3d R_1_2;
  double c2 = std::cos(q[1]);
  double s2 = std::sin(q[1]);
  R_1_2 << c2, -s2, 0.0, 0.0, 0.0, 1.0, -s2, -c2, 0.0;
  Eigen::Matrix3d R_2 = R_1 * R_1_2;
  Eigen::Vector3d x_2_3 = R_2.transpose() * x_3;
  q[2] = std::atan2(x_2_3[2], x_2_3[0]);

  if (q[2] <= q_min[2] || q[2] >= q_max[2]) return q_NAN;

  // IK: compute q5
  Eigen::Vector3d VH4 = p_2 + d3 * z_3 + a4 * x_3 - p_6 + d5 * z_5;
  Eigen::Matrix3d R_5_6;
  double c6 = std::cos(q[5]);
  double s6 = std::sin(q[5]);
  R_5_6 << c6, -s6, 0.0, 0.0, 0.0, -1.0, s6, c6, 0.0;
  Eigen::Matrix3d R_5 = R_6 * R_5_6.transpose();
  Eigen::Vector3d V_5_H4 = R_5.transpose() * VH4;

  q[4] = -std::atan2(V_5_H4[1], V_5_H4[0]);
  if (q[4] <= q_min[4] || q[4] >= q_max[4]) return q_NAN;

  return q;
}

Vector7d ik_sample_q7(Eigen::Matrix<double, 4, 4> O_T_EE, Vector7d q_actual,
                      bool is_fr3 = false, const size_t sample_size = 30,
                      const double sample_interval = 10) {
  const std::array<double, 7> q_min = is_fr3 ? q_min_fr3 : q_min_panda;
  const std::array<double, 7> q_max = is_fr3 ? q_max_fr3 : q_max_panda;

  auto q7 = q_actual[6];

  // sample interval in degree
  auto low7 = q7 - (sample_interval / 2.0) / 180.0 * M_PI;
  auto high7 = q7 + (sample_interval / 2.0) / 180.0 * M_PI;

  auto sample_step_size = (high7 - low7) / sample_size;

  Vector7d ik_solution =
      Vector7d::Constant(std::numeric_limits<double>::quiet_NaN());
  double loss = std::numeric_limits<double>::max();

  // shift if interval goes over the edges
  if (low7 < q_min[6]) {
    auto diff = q_min[6] - low7;
    low7 += diff;
    high7 += diff;
  } else if (high7 > q_max[6]) {
    auto diff = high7 - q_max[6];
    low7 -= diff;
    high7 -= diff;
  }
  for (size_t i = 0; i <= sample_size; ++i) {
    // last step is to try the current q7 position
    q7 = i < sample_size ? low7 + sample_step_size * i : q_actual[6];
    auto tmp_ik_sol = ik(O_T_EE, q_actual, q7, is_fr3);
    if (std::isnan(tmp_ik_sol[0])) {
      continue;
    }
    auto tmp_loss = (tmp_ik_sol - q_actual).norm();
    if (tmp_loss < loss) {
      loss = tmp_loss;
      ik_solution = tmp_ik_sol;
    }
  }
  return ik_solution;
}

Eigen::Matrix<double, 4, 4> fk(const Eigen::Matrix<double, 7, 1>& q) {
  const double d7e = 0.2104;
  Eigen::Matrix<double, 4, 4> pose;
  pose.setZero();

  // Aliases for brevity and safety
  double sin_q6_pi4 = std::sin(q[6] + M_PI_4);
  double cos_q6_pi4 = std::cos(q[6] + M_PI_4);

  // ROW 0
  pose.row(0)(0) =
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) *
          sin_q6_pi4 +
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[3]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * sin_q6_pi4 * std::cos(q[3]) *
          std::cos(q[4]) * std::cos(q[5]) -
      1.0 * std::sin(q[0]) * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[2]) *
          std::cos(q[5]) -
      1.0 * std::sin(q[0]) * std::cos(q[2]) * std::cos(q[4]) * cos_q6_pi4 -
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) * std::cos(q[0]) *
          cos_q6_pi4 +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * sin_q6_pi4 * std::cos(q[0]) *
          std::cos(q[4]) * std::cos(q[5]) +
      1.0 * std::sin(q[1]) * std::sin(q[5]) * sin_q6_pi4 * std::cos(q[0]) *
          std::cos(q[3]) -
      1.0 * std::sin(q[2]) * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[0]) *
          std::cos(q[1]) * std::cos(q[5]) -
      1.0 * std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[4]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[3]) * std::sin(q[5]) * sin_q6_pi4 * std::cos(q[0]) *
          std::cos(q[1]) * std::cos(q[2]) -
      1.0 * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) *
          std::cos(q[3]) * cos_q6_pi4 +
      1.0 * sin_q6_pi4 * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) *
          std::cos(q[3]) * std::cos(q[4]) * std::cos(q[5]);

  pose.row(0)(1) =
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) * sin_q6_pi4 *
          std::cos(q[3]) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::cos(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * cos_q6_pi4 -
      1.0 * std::sin(q[0]) * std::sin(q[4]) * std::cos(q[2]) * std::cos(q[5]) *
          cos_q6_pi4 +
      1.0 * std::sin(q[0]) * sin_q6_pi4 * std::cos(q[2]) * std::cos(q[4]) +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) * sin_q6_pi4 *
          std::cos(q[0]) +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0]) * std::cos(q[4]) *
          std::cos(q[5]) * cos_q6_pi4 +
      1.0 * std::sin(q[1]) * std::sin(q[5]) * std::cos(q[0]) * std::cos(q[3]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[1]) *
          std::cos(q[5]) * cos_q6_pi4 +
      1.0 * std::sin(q[2]) * sin_q6_pi4 * std::cos(q[0]) * std::cos(q[1]) *
          std::cos(q[4]) -
      1.0 * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[0]) * std::cos(q[1]) *
          std::cos(q[2]) * cos_q6_pi4 +
      1.0 * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[0]) * std::cos(q[1]) *
          std::cos(q[2]) * std::cos(q[3]) +
      1.0 * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) *
          std::cos(q[4]) * std::cos(q[5]) * cos_q6_pi4;

  pose.row(0)(2) = -1.0 *
                       (((std::sin(q[0]) * std::sin(q[2]) -
                          std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                             std::cos(q[3]) -
                         std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0])) *
                            std::cos(q[4]) +
                        (std::sin(q[0]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1])) *
                            std::sin(q[4])) *
                       std::sin(q[5]) -
                   1.0 *
                       ((std::sin(q[0]) * std::sin(q[2]) -
                         std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                            std::sin(q[3]) +
                        std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3])) *
                       std::cos(q[5]);

  pose.row(0)(3) = -d7e *
                       (((std::sin(q[0]) * std::sin(q[2]) -
                          std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                             std::cos(q[3]) -
                         std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0])) *
                            std::cos(q[4]) +
                        (std::sin(q[0]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1])) *
                            std::sin(q[4])) *
                       std::sin(q[5]) -
                   a7 *
                       (((std::sin(q[0]) * std::sin(q[2]) -
                          std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                             std::cos(q[3]) -
                         std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0])) *
                            std::cos(q[4]) +
                        (std::sin(q[0]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0]) * std::cos(q[1])) *
                            std::sin(q[4])) *
                       std::cos(q[5]) +
                   a7 *
                       ((std::sin(q[0]) * std::sin(q[2]) -
                         std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                            std::sin(q[3]) +
                        std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3])) *
                       std::sin(q[5]) -
                   d7e *
                       ((std::sin(q[0]) * std::sin(q[2]) -
                         std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                            std::sin(q[3]) +
                        std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3])) *
                       std::cos(q[5]) +
                   d5 *
                       (std::sin(q[0]) * std::sin(q[2]) -
                        std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                       std::sin(q[3]) +
                   a4 *
                       (std::sin(q[0]) * std::sin(q[2]) -
                        std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2])) *
                       std::cos(q[3]) -
                   a4 * std::sin(q[0]) * std::sin(q[2]) -
                   a4 * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[0]) +
                   d5 * std::sin(q[1]) * std::cos(q[0]) * std::cos(q[3]) +
                   d3 * std::sin(q[1]) * std::cos(q[0]) +
                   a4 * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]);

  // ROW 1
  pose.row(1)(0) =
      -1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) *
          cos_q6_pi4 +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * sin_q6_pi4 *
          std::cos(q[4]) * std::cos(q[5]) +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[5]) * sin_q6_pi4 *
          std::cos(q[3]) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) * sin_q6_pi4 *
          std::cos(q[1]) * std::cos(q[5]) -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) * std::cos(q[4]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[0]) * std::sin(q[3]) * std::sin(q[5]) * sin_q6_pi4 *
          std::cos(q[1]) * std::cos(q[2]) -
      1.0 * std::sin(q[0]) * std::sin(q[4]) * std::cos(q[1]) * std::cos(q[2]) *
          std::cos(q[3]) * cos_q6_pi4 +
      1.0 * std::sin(q[0]) * sin_q6_pi4 * std::cos(q[1]) * std::cos(q[2]) *
          std::cos(q[3]) * std::cos(q[4]) * std::cos(q[5]) -
      1.0 * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) * sin_q6_pi4 *
          std::cos(q[0]) -
      1.0 * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[3]) *
          cos_q6_pi4 +
      1.0 * std::sin(q[2]) * sin_q6_pi4 * std::cos(q[0]) * std::cos(q[3]) *
          std::cos(q[4]) * std::cos(q[5]) +
      1.0 * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[0]) * std::cos(q[2]) *
          std::cos(q[5]) +
      1.0 * std::cos(q[0]) * std::cos(q[2]) * std::cos(q[4]) * cos_q6_pi4;

  pose.row(1)(1) =
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[4]) *
          sin_q6_pi4 +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * cos_q6_pi4 +
      1.0 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[5]) * std::cos(q[3]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[0]) * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[1]) *
          std::cos(q[5]) * cos_q6_pi4 +
      1.0 * std::sin(q[0]) * std::sin(q[2]) * sin_q6_pi4 * std::cos(q[1]) *
          std::cos(q[4]) -
      1.0 * std::sin(q[0]) * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[1]) *
          std::cos(q[2]) * cos_q6_pi4 +
      1.0 * std::sin(q[0]) * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[1]) *
          std::cos(q[2]) * std::cos(q[3]) +
      1.0 * std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) *
          std::cos(q[4]) * std::cos(q[5]) * cos_q6_pi4 -
      1.0 * std::sin(q[2]) * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[0]) *
          cos_q6_pi4 +
      1.0 * std::sin(q[2]) * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[0]) *
          std::cos(q[3]) +
      1.0 * std::sin(q[2]) * std::cos(q[0]) * std::cos(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * cos_q6_pi4 +
      1.0 * std::sin(q[4]) * std::cos(q[0]) * std::cos(q[2]) * std::cos(q[5]) *
          cos_q6_pi4 -
      1.0 * sin_q6_pi4 * std::cos(q[0]) * std::cos(q[2]) * std::cos(q[4]);

  pose.row(1)(2) = 1.0 *
                       (((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                          std::sin(q[2]) * std::cos(q[0])) *
                             std::cos(q[3]) +
                         std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3])) *
                            std::cos(q[4]) -
                        (std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) -
                         std::cos(q[0]) * std::cos(q[2])) *
                            std::sin(q[4])) *
                       std::sin(q[5]) +
                   1.0 *
                       ((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0])) *
                            std::sin(q[3]) -
                        std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3])) *
                       std::cos(q[5]);

  pose.row(1)(3) = d7e *
                       (((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                          std::sin(q[2]) * std::cos(q[0])) *
                             std::cos(q[3]) +
                         std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3])) *
                            std::cos(q[4]) -
                        (std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) -
                         std::cos(q[0]) * std::cos(q[2])) *
                            std::sin(q[4])) *
                       std::sin(q[5]) +
                   a7 *
                       (((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                          std::sin(q[2]) * std::cos(q[0])) *
                             std::cos(q[3]) +
                         std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3])) *
                            std::cos(q[4]) -
                        (std::sin(q[0]) * std::sin(q[2]) * std::cos(q[1]) -
                         std::cos(q[0]) * std::cos(q[2])) *
                            std::sin(q[4])) *
                       std::cos(q[5]) -
                   a7 *
                       ((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0])) *
                            std::sin(q[3]) -
                        std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3])) *
                       std::sin(q[5]) +
                   d7e *
                       ((std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                         std::sin(q[2]) * std::cos(q[0])) *
                            std::sin(q[3]) -
                        std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3])) *
                       std::cos(q[5]) -
                   d5 *
                       (std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                        std::sin(q[2]) * std::cos(q[0])) *
                       std::sin(q[3]) -
                   a4 *
                       (std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                        std::sin(q[2]) * std::cos(q[0])) *
                       std::cos(q[3]) -
                   a4 * std::sin(q[0]) * std::sin(q[1]) * std::sin(q[3]) +
                   d5 * std::sin(q[0]) * std::sin(q[1]) * std::cos(q[3]) +
                   d3 * std::sin(q[0]) * std::sin(q[1]) +
                   a4 * std::sin(q[0]) * std::cos(q[1]) * std::cos(q[2]) +
                   a4 * std::sin(q[2]) * std::cos(q[0]);

  // ROW 2
  pose.row(2)(0) =
      1.0 * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) * sin_q6_pi4 *
          std::cos(q[5]) +
      1.0 * std::sin(q[1]) * std::sin(q[2]) * std::cos(q[4]) * cos_q6_pi4 +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[5]) * sin_q6_pi4 *
          std::cos(q[2]) +
      1.0 * std::sin(q[1]) * std::sin(q[4]) * std::cos(q[2]) * std::cos(q[3]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[1]) * sin_q6_pi4 * std::cos(q[2]) * std::cos(q[3]) *
          std::cos(q[4]) * std::cos(q[5]) -
      1.0 * std::sin(q[3]) * std::sin(q[4]) * std::cos(q[1]) * cos_q6_pi4 +
      1.0 * std::sin(q[3]) * sin_q6_pi4 * std::cos(q[1]) * std::cos(q[4]) *
          std::cos(q[5]) +
      1.0 * std::sin(q[5]) * sin_q6_pi4 * std::cos(q[1]) * std::cos(q[3]);

  pose.row(2)(1) =
      1.0 * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4]) * std::cos(q[5]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[1]) * std::sin(q[2]) * sin_q6_pi4 * std::cos(q[4]) +
      1.0 * std::sin(q[1]) * std::sin(q[3]) * std::sin(q[5]) * std::cos(q[2]) *
          cos_q6_pi4 -
      1.0 * std::sin(q[1]) * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[2]) *
          std::cos(q[3]) -
      1.0 * std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) * std::cos(q[4]) *
          std::cos(q[5]) * cos_q6_pi4 +
      1.0 * std::sin(q[3]) * std::sin(q[4]) * sin_q6_pi4 * std::cos(q[1]) +
      1.0 * std::sin(q[3]) * std::cos(q[1]) * std::cos(q[4]) * std::cos(q[5]) *
          cos_q6_pi4 +
      1.0 * std::sin(q[5]) * std::cos(q[1]) * std::cos(q[3]) * cos_q6_pi4;

  pose.row(2)(2) = -1.0 *
                       ((std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                         std::sin(q[3]) * std::cos(q[1])) *
                            std::cos(q[4]) -
                        std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4])) *
                       std::sin(q[5]) -
                   1.0 *
                       (std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
                        std::cos(q[1]) * std::cos(q[3])) *
                       std::cos(q[5]);

  pose.row(2)(3) = -d7e *
                       ((std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                         std::sin(q[3]) * std::cos(q[1])) *
                            std::cos(q[4]) -
                        std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4])) *
                       std::sin(q[5]) -
                   a7 *
                       ((std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                         std::sin(q[3]) * std::cos(q[1])) *
                            std::cos(q[4]) -
                        std::sin(q[1]) * std::sin(q[2]) * std::sin(q[4])) *
                       std::cos(q[5]) +
                   a7 *
                       (std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
                        std::cos(q[1]) * std::cos(q[3])) *
                       std::sin(q[5]) -
                   d7e *
                       (std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
                        std::cos(q[1]) * std::cos(q[3])) *
                       std::cos(q[5]) +
                   d5 * std::sin(q[1]) * std::sin(q[3]) * std::cos(q[2]) +
                   a4 * std::sin(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                   a4 * std::sin(q[1]) * std::cos(q[2]) -
                   a4 * std::sin(q[3]) * std::cos(q[1]) +
                   d5 * std::cos(q[1]) * std::cos(q[3]) + d3 * std::cos(q[1]) +
                   d1;

  pose.row(3)(3) = 1.0;
  return pose;
}

}  // namespace frankik

#endif  // FRANKIK_H