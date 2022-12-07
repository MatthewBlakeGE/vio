#ifndef LOCAL_PARAMETERIZATION_H
#define LOCAL_PARAMETERIZATION_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "common/common.h"

class LocalParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double* x, const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 7; };
  virtual int LocalSize() const { return 6; };
};

#endif  // LOCAL_PARAMETERIZATION_H