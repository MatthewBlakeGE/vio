#include "common/common.h"

Eigen::Matrix3d Common::g2R(const Eigen::Vector3d& g) {
  Eigen::Matrix3d R0;
  Eigen::Vector3d ng1 = g.normalized();
  Eigen::Vector3d ng2{0, 0, 1.0};
  R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
  double yaw = Common::R2ypr(R0).x();
  R0 = Common::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  // R0 = Common::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
  return R0;
}