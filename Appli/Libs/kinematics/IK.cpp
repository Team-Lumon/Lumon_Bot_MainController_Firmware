#include "eiquadprog.hpp"
#include "ik_wrapper.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace Eigen;

// --- System Configuration Constants ---
const float T_MOVE = 7.0f;
const float MASS = 2.0f;
const float GRAVITY = 9.81f;
const float R_PULLEY = 0.015f;
const float T_MIN = 2.0f;
const float T_MAX = 50.0f;
const float FRAME_DT = 0.1f;

// Minimum acceptable smallest singular value of W before flagging near-singular
// geometry. TUNE THIS empirically for your rig - start conservative and adjust
// based on real behavior.
const float MIN_SINGULAR_VALUE = 0.001f;

constexpr float HALF_PI_F = 1.5707963f;

const float T_MOVE_INV = 1.0f / T_MOVE;
const float T_MOVE_SQ_INV = 1.0f / (T_MOVE * T_MOVE);
const float T_MOVE_CB_INV = 1.0f / (T_MOVE * T_MOVE * T_MOVE);

Matrix<float, 3, 8> A;
Matrix<float, 3, 8> B;

struct MotorCommand {
  float L_total;
  float L_dot;
  float tau;
  bool feasible;
};

extern "C" void initGeometry() {
  // --- A: Fixed pulley pivots ---
  // NOTE: this uses a CORNER-origin frame (0 to 0.92m), not the center-origin
  // (0,0,0) convention discussed earlier. Confirm this is intentional before
  // trusting r_start/r_end below - they must be expressed in this SAME
  // corner-origin frame to be consistent.
  const float cube_sz = 0.92f;
  const float xy_off = 0.026f;
  const float z_top = cube_sz - 0.062f; // 0.858 m
  const float z_mid = 0.07f;

  // UNCONFIRMED: assumes the middle-layer pulleys use the same 2.6cm X/Y inset
  // as the top pulleys. You flagged this as an open question - verify against
  // your actual rig.
  A.col(0) << xy_off, xy_off, z_top;
  A.col(1) << cube_sz - xy_off, xy_off, z_top;
  A.col(2) << cube_sz - xy_off, cube_sz - xy_off, z_top;
  A.col(3) << xy_off, cube_sz - xy_off, z_top;

  A.col(4) << xy_off, xy_off, z_mid;
  A.col(5) << cube_sz - xy_off, xy_off, z_mid;
  A.col(6) << cube_sz - xy_off, cube_sz - xy_off, z_mid;
  A.col(7) << xy_off, cube_sz - xy_off, z_mid;

  // --- B: End-effector cable anchor points, relative to its own center ---
  // STILL PLACEHOLDER - replace with your real measured end-effector
  // dimensions.
  // length = 13cm
  // width = 11cm
  // height  = 6cm
  const float hx = 0.055f; // half of 0.11m width
  const float hy = 0.065f; // half of 0.13m length
  const float hz = 0.03f;  // half of 0.06m height

  B.col(0) << -hx, -hy, hz;
  B.col(1) << hx, -hy, hz;
  B.col(2) << hx, hy, hz;
  B.col(3) << -hx, hy, hz;
  B.col(4) << -hx, -hy, -hz;
  B.col(5) << hx, -hy, -hz;
  B.col(6) << hx, hy, -hz;
  B.col(7) << -hx, hy, -hz;
}

void computeFrameTargets(float t, const Vector3f &r_start,
                         const Vector3f &r_end, MotorCommand *commands,
                         Vector3f &r_out, Vector3f &r_dot_out) {

  static Matrix<float, 8, 1> last_good_tau =
      Matrix<float, 8, 1>::Constant((T_MIN + T_MAX) / 2.0f);

  if (t > T_MOVE)
    t = T_MOVE;

  float t_sq = t * t;
  float t_ratio = t * T_MOVE_INV;
  float t_ratio_sq = t_ratio * t_ratio;
  float t_ratio_cb = t_ratio_sq * t_ratio;

  float s = 3.0f * t_ratio_sq - 2.0f * t_ratio_cb;
  float s_dot = (6.0f * t * T_MOVE_SQ_INV) - (6.0f * t_sq * T_MOVE_CB_INV);
  float s_ddot = (6.0f * T_MOVE_SQ_INV) - (12.0f * t * T_MOVE_CB_INV);

  Vector3f delta_r = r_end - r_start;
  Vector3f r = r_start + (s * delta_r);
  Vector3f r_dot = s_dot * delta_r;
  Vector3f r_ddot = s_ddot * delta_r;

  r_out = r;
  r_dot_out = r_dot;

  Matrix<float, 6, 1> f_dyn;
  f_dyn << MASS * r_ddot.x(), MASS * r_ddot.y(), MASS * (r_ddot.z() + GRAVITY),
      0.0f, 0.0f, 0.0f;

  Matrix<float, 6, 8> W;

  for (int i = 0; i < 8; i++) {
    Vector3f P_i = r + B.col(i);
    Vector3f P_dot_i = r_dot;

    float dx = P_i.x() - A.col(i).x();
    float dy = P_i.y() - A.col(i).y();
    float d_xy = sqrtf(dx * dx + dy * dy);

    float theta = atan2f(dy, dx);

    Vector3f a_exit;
    a_exit << A.col(i).x() + R_PULLEY * cosf(theta),
        A.col(i).y() + R_PULLEY * sinf(theta), A.col(i).z();

    Vector3f l_free_vec = a_exit - P_i;
    float L_free = l_free_vec.norm();
    Vector3f u_hat = l_free_vec / L_free;

    float alpha = atan2f(P_i.z() - A.col(i).z(), d_xy - R_PULLEY);
    float L_wrap = R_PULLEY * (HALF_PI_F - alpha);

    commands[i].L_total = L_free + L_wrap;

    float L_dot_free = -(u_hat.dot(P_dot_i));

    float d_xy_dot = (dx * P_dot_i.x() + dy * P_dot_i.y()) / d_xy;
    float d_xy_min_R = d_xy - R_PULLEY;
    float dz = P_i.z() - A.col(i).z();

    float denom = (d_xy_min_R * d_xy_min_R) + (dz * dz);
    float alpha_dot = (d_xy_min_R * P_dot_i.z() - dz * d_xy_dot) / denom;
    float L_dot_wrap = -R_PULLEY * alpha_dot;

    commands[i].L_dot = L_dot_free + L_dot_wrap;

    Vector3f torque_component = B.col(i).cross(u_hat);
    W.col(i) << u_hat.x(), u_hat.y(), u_hat.z(), torque_component.x(),
        torque_component.y(), torque_component.z();
  }

  // --- Conditioning check on W directly (not WWt's determinant, which is
  // scale-distorted) ---
  JacobiSVD<Matrix<float, 6, 8>> svd(W);
  float min_singular_value = svd.singularValues().tail(1)(0);
  bool singular = (min_singular_value < MIN_SINGULAR_VALUE);

  Matrix<float, 8, 6> W_pinv;
  Matrix<float, 8, 1> tau;

  if (singular) {
    tau = last_good_tau;
    for (int i = 0; i < 8; i++) {
      commands[i].tau = tau(i);
      commands[i].feasible = false;
    }
    return;
  }

  // Set up QP problem:
  // min 0.5 * x^T G x + g0^T x
  // s.t. CE^T x + ce0 = 0
  //      CI^T x + ci0 >= 0

  Eigen::MatrixXd G = Eigen::MatrixXd::Identity(8, 8);
  // tau_mid = 25.0f, so we want to minimize 0.5 ||tau - 25||^2
  // 0.5 * (x - 25)^T (x - 25) = 0.5 * x^T x - 25^T x + C
  Eigen::VectorXd g0 = -25.0 * Eigen::VectorXd::Ones(8);

  // Equality constraints: W * tau = f_dyn
  // eiquadprog expects CE^T x + ce0 = 0, so CE^T = W => CE = W^T
  Eigen::MatrixXd CE = W.transpose().cast<double>();
  Eigen::VectorXd ce0 = -f_dyn.cast<double>();

  // Inequality constraints: CI^T * x + ci0 >= 0
  // x >= T_MIN => x - T_MIN >= 0
  // x <= T_MAX => -x + T_MAX >= 0
  Eigen::MatrixXd CI(8, 16);
  CI.leftCols(8) = Eigen::MatrixXd::Identity(8, 8);
  CI.rightCols(8) = -Eigen::MatrixXd::Identity(8, 8);

  Eigen::VectorXd ci0(16);
  ci0.head(8) = -T_MIN * Eigen::VectorXd::Ones(8);
  ci0.tail(8) = T_MAX * Eigen::VectorXd::Ones(8);

  Eigen::VectorXd x(8);
  Eigen::VectorXi activeSet;
  size_t activeSetSize;

  double cost = eiquadprog::solvers::solve_quadprog(G, g0, CE, ce0, CI, ci0, x,
                                                     activeSet, activeSetSize);

  bool feasible = !std::isinf(cost);

  if (feasible) {
    tau = x.cast<float>();
  } else {
    tau = last_good_tau;
  }

  for (int i = 0; i < 8; i++) {
    commands[i].tau = tau(i);
    commands[i].feasible = feasible;
  }

  if (feasible) {
    last_good_tau = tau;
  }
}

void sendToMotorPCBs(const MotorCommand *commands) {
  for (int i = 0; i < 8; i++) {
    // pack + transmit over your UART/CAN/SPI here
  }
}

void printFrame(float t, const Vector3f &r, const Vector3f &r_dot,
                const MotorCommand *commands) {
  printf("---- t = %.2f s ----\n", t);
  printf("Target Pos : [%.4f, %.4f, %.4f] m\n", r.x(), r.y(), r.z());
  printf("Target Vel : [%.4f, %.4f, %.4f] m/s\n", r_dot.x(), r_dot.y(),
         r_dot.z());
  printf("Cable Lengths (m): ");
  for (int i = 0; i < 8; i++) {
    printf("%.4f ", commands[i].L_total);
  }
  printf("\nCable Vel (m/s):   ");
  for (int i = 0; i < 8; i++) {
    printf("%+.4f ", commands[i].L_dot);
  }
  printf("\nTensions (N):      ");
  for (int i = 0; i < 8; i++) {
    printf("%.2f ", commands[i].tau);
  }
  printf("  [feasible=%s]\n\n", commands[0].feasible ? "true" : "false");
}

extern "C" {

void computeFrameTargetsWrapper(float t, const Vector3_t *r_start,
                                const Vector3_t *r_end,
                                MotorCommand_t *commands, Vector3_t *r_out,
                                Vector3_t *r_dot_out) {

  Vector3f start(r_start->x, r_start->y, r_start->z);
  Vector3f end(r_end->x, r_end->y, r_end->z);

  MotorCommand cpp_commands[8];
  Vector3f out_pos, out_vel;

  computeFrameTargets(t, start, end, cpp_commands, out_pos, out_vel);

  if (r_out) {
    r_out->x = out_pos.x();
    r_out->y = out_pos.y();
    r_out->z = out_pos.z();
  }

  if (r_dot_out) {
    r_dot_out->x = out_vel.x();
    r_dot_out->y = out_vel.y();
    r_dot_out->z = out_vel.z();
  }

  if (commands) {
    for (int i = 0; i < 8; i++) {
      commands[i].L_total = cpp_commands[i].L_total;
      commands[i].L_dot = cpp_commands[i].L_dot;
      commands[i].tau = cpp_commands[i].tau;
      commands[i].feasible = cpp_commands[i].feasible;
    }
  }
}

} // extern "C"
