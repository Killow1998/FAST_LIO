#pragma once
#include "imu_initialization.h"
#include "use-ikfom.hpp"
#include <common_lib.h>

class ImuProcess {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess() = default;

  void Reset();
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
               PointCloudXYZI::Ptr pcl_un_, const fast_lio::OdometryObserver &observer = {});

  bool initializing() const { return imu_need_init_; }
  void setGravity(double gravity) { gravity_m_s2_ = gravity; }
  std::string report;

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int lidar_type;

private:
  bool IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);
  void UndistortPcl(const MeasureGroup &meas,
                    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                    PointCloudXYZI &pcl_in_out, const fast_lio::OdometryObserver &observer);

  std::shared_ptr<const fast_lio::ImuSample> last_imu_;
  vector<Pose6D> IMUpose;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  fast_lio::ImuInitializationWindow initialization_window_;
  double initialization_report_time_ = -1.;
  unsigned initialization_reports_ = 0;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double last_lidar_end_time_ = 0.;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;

  double gravity_m_s2_ = 9.81;
};
