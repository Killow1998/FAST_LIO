#include "IMU_Processing.hpp"
#include <cstdio>

ImuProcess::ImuProcess() {
  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new fast_lio::ImuSample());
}

void ImuProcess::Reset() {
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  imu_need_init_ = true;
  b_first_frame_ = true;
  initialization_window_.reset();
  initialization_report_time_ = -1.;
  initialization_reports_ = 0;
  last_lidar_end_time_ = 0.;
  IMUpose.clear();
  last_imu_.reset(new fast_lio::ImuSample());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T) {
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D &scaler) { cov_acc_scale = scaler; }

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

bool ImuProcess::IMU_init(
    const MeasureGroup &meas,
    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state) {
  if (b_first_frame_) {
    Reset();
    b_first_frame_ = false;
    first_lidar_time = meas.lidar_beg_time;
  }
  for (const auto &imu : meas.imu) {
    const auto &a = imu->linear_acceleration;
    const auto &w = imu->angular_velocity;
    initialization_window_.add(imu->seconds(), V3D(a.x(), a.y(), a.z()),
                               V3D(w.x(), w.y(), w.z()));
  }
  const auto estimate = initialization_window_.estimate();
  if (!estimate.ready) {
    if (initialization_reports_ < 10 &&
        (initialization_report_time_ < 0. ||
         meas.lidar_end_time < initialization_report_time_ ||
         meas.lidar_end_time - initialization_report_time_ >= 1.)) {
      char message[512];
      std::snprintf(message, sizeof(message),
                    "IMU initialization waiting: samples=%zu window_s=%.3f "
                    "acc_std_g=%.6f "
                    "gyro_rms_rad_s=%.6f acc_mean_change_g=%.6f "
                    "gyro_mean_change_rad_s=%.6f",
                    estimate.samples, estimate.duration_s,
                    estimate.acceleration_std_g, estimate.angular_rate_rms,
                    estimate.acceleration_mean_change_g,
                    estimate.angular_rate_mean_change);
      report = message;
      initialization_report_time_ = meas.lidar_end_time;
      ++initialization_reports_;
    }
    return false;
  }
  mean_acc = estimate.acceleration;
  mean_gyr = estimate.angular_rate;
  cov_acc = estimate.acceleration_variance;
  cov_gyr = estimate.angular_rate_variance;
  state_ikfom init_state = kf_state.get_x();
  V3D meas_grav_in_body = mean_acc;
  V3D target_grav_in_world(0, 0, 1.0);

  Eigen::Quaterniond q_init;
  q_init.setFromTwoVectors(meas_grav_in_body.normalized(),
                           target_grav_in_world);

  init_state.rot = SO3(q_init.toRotationMatrix());
  // The S2 constructor normalizes to its template radius (9.809), so set both
  // the runtime radius and vector to the configured acceleration scale.
  init_state.grav.length = gravity_m_s2_;
  init_state.grav.vec = V3D(0, 0, -gravity_m_s2_);
  init_state.pos = V3D(0, 0, 0);
  init_state.vel = Zero3d;

  init_state.bg = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  const double grav_dir_var = estimate.gravityDirectionVariance();
  init_P(21, 21) = init_P(22, 22) = std::max(grav_dir_var, 0.00001);
  char message[512];
  std::snprintf(message, sizeof(message),
                "IMU Initial Done: samples=%zu window_s=%.3f acc_std_g=%.6f "
                "gyro_rms_rad_s=%.6f "
                "grav_dir_var=%.3e",
                estimate.samples, estimate.duration_s,
                estimate.acceleration_std_g, estimate.angular_rate_rms,
                init_P(21, 21));
  report = message;
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = meas.lidar_end_time;
  const auto &a = last_imu_->linear_acceleration;
  const auto &w = last_imu_->angular_velocity;
  angvel_last = V3D(w.x(), w.y(), w.z()) - mean_gyr;
  acc_s_last = init_state.rot * (V3D(a.x(), a.y(), a.z()) * gravity_m_s2_ /
                                 mean_acc.norm()) +
               V3D(0., 0., -gravity_m_s2_);
  initialization_window_.reset();
  return true;
}
