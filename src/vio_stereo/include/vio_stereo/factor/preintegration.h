#ifndef INTEGRATION_H
#define INTEGRATION_H

#include "common/common.h"
#include "common/parameters.h"

#include <ceres/ceres.h>
using namespace Eigen;

class Preintegration {
 public:
  Preintegration() = delete;
  Preintegration(const Eigen::Vector3d& _acc_0, const Eigen::Vector3d& _gyr_0,
                 const Eigen::Vector3d& _linearized_ba,
                 const Eigen::Vector3d& _linearized_bg);

  void push_back(double dt, const Eigen::Vector3d& acc,
                 const Eigen::Vector3d& gyr);

  void repropagate(const Eigen::Vector3d& _linearized_ba,
                   const Eigen::Vector3d& _linearized_bg);

  void midPointIntegration(
      double _dt, const Eigen::Vector3d& _acc_0, const Eigen::Vector3d& _gyr_0,
      const Eigen::Vector3d& _acc_1, const Eigen::Vector3d& _gyr_1,
      const Eigen::Vector3d& delta_p, const Eigen::Quaterniond& delta_q,
      const Eigen::Vector3d& delta_v, const Eigen::Vector3d& linearized_ba,
      const Eigen::Vector3d& linearized_bg, Eigen::Vector3d& result_delta_p,
      Eigen::Quaterniond& result_delta_q, Eigen::Vector3d& result_delta_v,
      Eigen::Vector3d& result_linearized_ba,
      Eigen::Vector3d& result_linearized_bg, bool update_jacobian);

  void propagate(double _dt, const Eigen::Vector3d& _acc_1,
                 const Eigen::Vector3d& _gyr_1);

  Eigen::Matrix<double, 15, 1> evaluate(
      const Eigen::Vector3d& Pi, const Eigen::Quaterniond& Qi,
      const Eigen::Vector3d& Vi, const Eigen::Vector3d& Bai,
      const Eigen::Vector3d& Bgi, const Eigen::Vector3d& Pj,
      const Eigen::Quaterniond& Qj, const Eigen::Vector3d& Vj,
      const Eigen::Vector3d& Baj, const Eigen::Vector3d& Bgj);

  double dt;
  Eigen::Vector3d acc_0, gyr_0;
  Eigen::Vector3d acc_1, gyr_1;

  const Eigen::Vector3d linearized_acc, linearized_gyr;
  Eigen::Vector3d linearized_ba, linearized_bg;

  Eigen::Matrix<double, 15, 15> jacobian, covariance;
  Eigen::Matrix<double, 15, 15> step_jacobian;
  Eigen::Matrix<double, 15, 18> step_V;
  //   Eigen::DiagonalMatrix<double, 18, 18> noise;
  Eigen::Matrix<double, 18, 18> noise;

  double sum_dt;
  Eigen::Vector3d delta_p;
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_v;

  std::vector<double> dt_buf;
  std::vector<Eigen::Vector3d> acc_buf;
  std::vector<Eigen::Vector3d> gyr_buf;
};

// void eulerIntegration(double _dt, const Eigen::Vector3d &_acc_0, const
// Eigen::Vector3d &_gyr_0,
//                         const Eigen::Vector3d &_acc_1, const Eigen::Vector3d
//                         &_gyr_1, const Eigen::Vector3d &delta_p, const
//                         Eigen::Quaterniond &delta_q, const Eigen::Vector3d
//                         &delta_v, const Eigen::Vector3d &linearized_ba, const
//                         Eigen::Vector3d &linearized_bg, Eigen::Vector3d
//                         &result_delta_p, Eigen::Quaterniond &result_delta_q,
//                         Eigen::Vector3d &result_delta_v, Eigen::Vector3d
//                         &result_linearized_ba, Eigen::Vector3d
//                         &result_linearized_bg, bool update_jacobian);

// void checkJacobian(double _dt, const Eigen::Vector3d &_acc_0, const
// Eigen::Vector3d &_gyr_0,
//                                const Eigen::Vector3d &_acc_1, const
//                                Eigen::Vector3d &_gyr_1,
//                         const Eigen::Vector3d &delta_p, const
//                         Eigen::Quaterniond &delta_q, const Eigen::Vector3d
//                         &delta_v, const Eigen::Vector3d &linearized_ba, const
//                         Eigen::Vector3d &linearized_bg);

#endif  // INTEGRATION_H