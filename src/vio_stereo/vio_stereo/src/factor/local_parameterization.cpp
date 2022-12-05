#include "factor/local_parameterization.h"

// 定义了四元数的加法
bool LocalParameterization::Plus(const double* x, const double* delta,
                                 double* x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  Eigen::Map<const Eigen::Vector3d> dp(delta);

  Eigen::Quaterniond dq =
      Common::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  p = _p + dp;
  q = (_q * dq).normalized();

  return true;
}

//FIXME: 计算新的jocabian矩阵
bool LocalParameterization::ComputeJacobian(const double* x,
                                            double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  // Eigen::RowMajor指的是行优先
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();  //初始化j

  return true;
}
