#ifndef INITIAL_BIAS_FACTOR_H
#define INITIAL_BIAS_FACTOR_H

#include <ceres/ceres.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include "common/common.h"
#include "common/parameters.h"
#include "common/time_cost.h"

class InitialBiasFactor : public ceres::SizedCostFunction<6, 9> {
 public:
  InitialBiasFactor(const Eigen::Vector3d& _Ba, const Eigen::Vector3d& _Bg) {
    init_Ba = _Ba;
    init_Bg = _Bg;
    sqrt_info = 1.0 / (0.001) * Eigen::Matrix<double, 6, 6>::Identity();
  }
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    Eigen::Vector3d Ba(parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d Bg(parameters[0][6], parameters[0][7], parameters[0][8]);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.block<3, 1>(0, 0) = Ba - init_Ba;
    residual.block<3, 1>(3, 0) = Bg - init_Bg;
    residual = sqrt_info * residual;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 9, Eigen::RowMajor>> jacobian_bias(
            jacobians[0]);
        jacobian_bias.setZero();
        jacobian_bias.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        jacobian_bias.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
        jacobian_bias = sqrt_info * jacobian_bias;
      }
    }
    return true;
  }

  Eigen::Vector3d init_Ba, init_Bg;
  Eigen::Matrix<double, 6, 6> sqrt_info;
};

#endif  // INITIAL_BIAS_FACTOR_H