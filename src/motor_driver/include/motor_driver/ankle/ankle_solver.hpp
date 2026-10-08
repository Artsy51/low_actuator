#ifndef MOTOR_DRIVER_ANKLE_SOLVER_HPP
#define MOTOR_DRIVER_ANKLE_SOLVER_HPP

#include <Eigen/Dense>
#include <cmath>
#include <limits>

// Generated solver exports a C ABI so it can be called from C++.
extern "C"
{
int gen_ik(const double **arg, double **res, long long *iw, double *w, int mem);
int gen_jac(const double **arg, double **res, long long *iw, double *w, int mem);
}

class Ankle_Solver
{
  public:
  Ankle_Solver() = default;
  enum class FkError
  {
    None,
    InvalidIk,
    InvalidJacobian,
    InvalidUpdate,
    MaxIterations,
  };

  struct FkResult
  {
    Eigen::Vector2d position = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    double residual = std::numeric_limits<double>::infinity();
    int iterations = 0;

    FkError error = FkError::MaxIterations;

    [[nodiscard]] bool ok() const noexcept
    {
      return error == FkError::None;
    }
  };

  [[nodiscard]] FkResult fk_pose(const Eigen::Vector2d &q_ref,
                                 const Eigen::Vector2d &p_init = Eigen::Vector2d(0.0, 0.0),
                                 int max_iter = 20, double tol = 1e-6)
  {
    Eigen::Vector2d p = p_init;
    FkResult result;

    for (int i = 0; i < max_iter; ++i)
    {
      result.iterations = i + 1;

      // 1. 正向计算当前的电机角度 q_curr = IK(p)
      Eigen::Vector2d q_curr = ik_pose(p(0), p(1));
      if (!q_curr.allFinite())
      {
        result.error = FkError::InvalidIk;
        return result;
      }

      // 2. 计算误差
      Eigen::Vector2d q_err = q_curr - q_ref;
      result.residual = q_err.norm();

      // 3. 检查收敛
      if (result.residual < tol)
      {
        result.position = p;
        result.error = FkError::None;
        return result;
      }

      // 4. 计算雅可比矩阵
      Eigen::Matrix2d J = jacobian(p(0), p(1));
      if (!J.allFinite())
      {
        result.error = FkError::InvalidJacobian;
        return result;
      }

      // Limit a Newton step so a distant seed cannot jump across branches.
      Eigen::Matrix2d J_inv = fast_2x2_inverse(J);
      if (!J_inv.allFinite())
      {
        result.error = FkError::InvalidJacobian;
        return result;
      }

      Eigen::Vector2d step = J_inv * q_err;
      if (!step.allFinite())
      {
        result.error = FkError::InvalidUpdate;
        return result;
      }

      constexpr double max_step = 0.5;
      const double step_norm_sq = step.squaredNorm();
      if (step_norm_sq > max_step * max_step)
        step *= max_step / std::sqrt(step_norm_sq);
      p -= step;
      if (!p.allFinite())
      {
        result.error = FkError::InvalidUpdate;
        return result;
      }
    }

    return result;
  }

  // --- 1. 正运动学 (速度): fk_vel ---
  Eigen::Vector2d fk_vel(const Eigen::Vector2d &q_dot_rad_per_s, const Eigen::Vector2d &p_current_rad)
  {
    if (!q_dot_rad_per_s.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    // A. 计算雅可比
    Eigen::Matrix2d J = jacobian(p_current_rad(0), p_current_rad(1));
    if (!J.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    // B. 快速求逆 (DLS保护)
    Eigen::Matrix2d J_inv = fast_2x2_inverse(J);
    if (!J_inv.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    // C. 求解
    Eigen::Vector2d p_dot_raw = J_inv * q_dot_rad_per_s;
    if (!p_dot_raw.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return p_dot_raw;
  }

  // 电机力矩 -> 踝关节力矩: tau_p = J^T * tau_q
  [[nodiscard]] Eigen::Vector2d fk_torque(const Eigen::Vector2d &motor_torque,
                                           const Eigen::Vector2d &p_current_rad)
  {
    if (!motor_torque.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    const Eigen::Matrix2d J = jacobian(p_current_rad(0), p_current_rad(1));
    if (!J.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    const Eigen::Vector2d ankle_torque = J.transpose() * motor_torque;
    return ankle_torque.allFinite()
               ? ankle_torque
               : Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  }

  Eigen::Vector2d ik_pose(double psi, double phi)
  {
    if (!std::isfinite(psi) || !std::isfinite(phi))
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    double output[2];

    const double *arg[2] = {&psi, &phi};
    double *res[1] = {output};
    int ret = gen_ik(arg, res, nullptr, nullptr, 0);
    if (ret != 0 || !std::isfinite(output[0]) || !std::isfinite(output[1]))
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    return Eigen::Vector2d(output[0], output[1]);
  }

  // 踝关节期望力矩 -> 电机期望力矩: tau_q = J^-T * tau_p
  [[nodiscard]] Eigen::Vector2d ik_torque(const Eigen::Vector2d &joint_torque,
                                           const Eigen::Vector2d &p_current_rad)
  {
    if (!joint_torque.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    const Eigen::Matrix2d J = jacobian(p_current_rad(0), p_current_rad(1));
    if (!J.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    const Eigen::Matrix2d J_inv_t = fast_2x2_inverse(J.transpose());
    if (!J_inv_t.allFinite())
    {
      return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    const Eigen::Vector2d motor_torque = J_inv_t * joint_torque;
    return motor_torque.allFinite()
               ? motor_torque
               : Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  }


  // --- 3. 雅可比矩阵: jacobian ---
  Eigen::Matrix2d jacobian(double psi, double phi)
  {
    if (!std::isfinite(psi) || !std::isfinite(phi))
    {
      return Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    double jac_out[4];

    const double *arg[2] = {&psi, &phi};
    double *res[1] = {jac_out};
    int ret = gen_jac(arg, res, nullptr, nullptr, 0);
    if (ret != 0)
    {
      return Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    Eigen::Matrix2d J;
    // 数据排布映射 (CasADi 列优先)
    J(0, 0) = jac_out[0];
    J(1, 0) = jac_out[1];
    J(0, 1) = jac_out[2];
    J(1, 1) = jac_out[3];
    if (!J.allFinite())
    {
      return Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return J;
  }

  private:
  double damping_sq = 1e-8;

  // --- 内部辅助: 2x2矩阵快速求逆 ---
  Eigen::Matrix2d fast_2x2_inverse(const Eigen::Matrix2d &A)
  {
    double det = A.determinant();

    // 如果远离奇异点，直接求逆
    const double scale = A.squaredNorm();
    if (std::abs(det) > 1e-6 * scale)
    {
      Eigen::Matrix2d inv = A.inverse();
      if (inv.allFinite())
      {
        return inv;
      }
    }

    // 奇异点保护 (Damped Least Squares)
    Eigen::Matrix2d At = A.transpose();
    Eigen::Matrix2d M = A * At;
    M(0, 0) += damping_sq;
    M(1, 1) += damping_sq;
    return At * M.inverse();
  }
};

#endif // MOTOR_DRIVER_ANKLE_SOLVER_HPP
