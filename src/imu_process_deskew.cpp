#include "IMU_Processing.hpp"
#include "odometry_output.h"
#include "preprocess.h"
#include <algorithm>

namespace {
bool time_list(const PointType &a, const PointType &b) { return a.curvature < b.curvature; }
} // namespace

void ImuProcess::UndistortPcl(const MeasureGroup &meas,
                              esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudXYZI &pcl_out, const fast_lio::OdometryObserver &observer) {

  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  double imu_beg_time = v_imu.front()->seconds();
  double imu_end_time = v_imu.back()->seconds();

  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;

  if (lidar_type == MARSIM) {
    pcl_beg_time = last_lidar_end_time_;
    pcl_end_time = meas.lidar_beg_time;
  }

  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos,
                               imu_state.rot.toRotationMatrix()));

  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (tail->seconds() < last_lidar_end_time_)
      continue;

    angvel_avr << 0.5 * (head->angular_velocity.x() + tail->angular_velocity.x()),
        0.5 * (head->angular_velocity.y() + tail->angular_velocity.y()),
        0.5 * (head->angular_velocity.z() + tail->angular_velocity.z());
    acc_avr << 0.5 * (head->linear_acceleration.x() + tail->linear_acceleration.x()),
        0.5 * (head->linear_acceleration.y() + tail->linear_acceleration.y()),
        0.5 * (head->linear_acceleration.z() + tail->linear_acceleration.z());

    acc_avr = acc_avr * gravity_m_s2_ / mean_acc.norm(); // - state_inout.ba;

    if (head->seconds() < last_lidar_end_time_) {
      dt = tail->seconds() - last_lidar_end_time_;
    } else {
      dt = tail->seconds() - head->seconds();
    }

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++) {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail->seconds() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos,
                                 imu_state.rot.toRotationMatrix()));

    if (observer)
      observer(fast_lio::OdometryKind::Imu,
               fast_lio::makeOdometry(tail->stamp_ns, imu_state, kf_state.get_P(),
                                      tail->angular_velocity));
  }

  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  if (pcl_out.points.begin() == pcl_out.points.end())
    return;

  if (lidar_type != MARSIM) {
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      acc_imu << VEC_FROM_ARRAY(tail->acc);
      angvel_avr << VEC_FROM_ARRAY(tail->gyr);

      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        // Transform the acquisition-time lidar point into the scan-end lidar frame.
        M3D R_i(R_imu * Exp(angvel_avr, dt));

        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
        V3D P_compensate =
            imu_state.offset_R_L_I.conjugate() *
            (imu_state.rot.conjugate() *
                 (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
             imu_state.offset_T_L_I);

        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_out.points.begin())
          break;
      }
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,
                         esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudXYZI::Ptr cur_pcl_un_,
                         const fast_lio::OdometryObserver &observer) {
  report.clear();

  if (meas.imu.empty()) {
    return;
  };
  assert(meas.lidar != nullptr);

  if (imu_need_init_) {
    cur_pcl_un_->clear();
    if (IMU_init(meas, kf_state)) {
      imu_need_init_ = false;
      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
    }
    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_, observer);

}
