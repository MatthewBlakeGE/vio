#ifndef INITIAL_EX_ROTATION_H
#define INITIAL_EX_ROTATION_H

#include <vector>
#include "common/parameters.h"
using namespace std;

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
using namespace Eigen;
#include <ros/console.h>

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally don't konw the extrinsic parameter */
// 当您完全不了解外部参数时，该类可帮助您校准imu和相机之间的外部旋转
class InitialEXRotation {
 public:
  InitialEXRotation();
  bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres,
                             Quaterniond delta_q_imu,
                             Matrix3d& calib_ric_result);

 private:
  Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>>& corres);

  double testTriangulation(const vector<cv::Point2f>& l,
                           const vector<cv::Point2f>& r, cv::Mat_<double> R,
                           cv::Mat_<double> t);
  void decomposeE(cv::Mat E, cv::Mat_<double>& R1, cv::Mat_<double>& R2,
                  cv::Mat_<double>& t1, cv::Mat_<double>& t2);
  int frame_count;

  vector<Matrix3d> Rc;
  vector<Matrix3d> Rimu;
  vector<Matrix3d> Rc_g;
  Matrix3d ric;
};

#endif  // INITIAL_EX_ROTATION_H
