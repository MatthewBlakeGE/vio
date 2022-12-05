#ifndef LOCAL_PARAMETERIZATION_H
#define LOCAL_PARAMETERIZATION_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "common/common.h"

// 对于四元数或者旋转矩阵这种使用过参数化表示旋转的方式，它们是不支持广义的加法
// 所以我们在使用ceres对其进行迭代更新的时候就需要自定义其更新方式了，具体的做法是实现一个LocalParameterization
class LocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const;  //定义了四元数的加法
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 7; };  //返回7
  virtual int LocalSize() const { return 6; };   //返回6
};

#endif  // LOCAL_PARAMETERIZATION_H