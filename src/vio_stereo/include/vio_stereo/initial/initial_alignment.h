#ifndef INITIAL_ALIGNMENT_H
#define INITIAL_ALIGNMENT_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include "common/common.h"
#include "estimator/feature_manager.h"
#include "factor/imu_factor.h"

using namespace Eigen;
using namespace std;
class ImageFrame {
 public:
  ImageFrame(){};
  ImageFrame(
      const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points,
      double _t)
      : t{_t}, is_key_frame{false} {
    points = _points;
  };
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;
  double t;
  Matrix3d R;
  Vector3d T;
  Preintegration* pre_integration;
  bool is_key_frame;
};
void solveGyroscopeBias(map<double, ImageFrame>& all_image_frame,
                        Vector3d* Bgs);
bool VisualIMUAlignment(map<double, ImageFrame>& all_image_frame, Vector3d* Bgs,
                        Vector3d& g, VectorXd& x);

#endif  // INITIAL_ALIGNMENT_H
