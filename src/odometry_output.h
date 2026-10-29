#pragma once

#include "fast_lio/types.h"
#include "use-ikfom.hpp"

namespace fast_lio {

inline Odometry makeOdometry(
    int64_t stamp_ns, const state_ikfom &state,
    const Eigen::Matrix<double, state_ikfom::DOF, state_ikfom::DOF> &covariance,
    const Eigen::Vector3d &measured_angular_velocity) {
  Odometry out;
  out.stamp_ns = stamp_ns;
  out.position = state.pos;
  out.orientation = Eigen::Quaterniond(state.rot);
  const Eigen::Matrix3d rotation = state.rot.toRotationMatrix();
  out.velocity = rotation.transpose() * state.vel;
  out.angular_velocity = measured_angular_velocity - state.bg;

  // IKFoM uses [position, right attitude perturbation]; ROS uses parent-frame axes.
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_covariance(
      out.covariance.data());
  pose_covariance.topLeftCorner<3, 3>() = covariance.topLeftCorner<3, 3>();
  pose_covariance.topRightCorner<3, 3>() = covariance.block<3, 3>(0, 3) * rotation.transpose();
  pose_covariance.bottomLeftCorner<3, 3>() = rotation * covariance.block<3, 3>(3, 0);
  pose_covariance.bottomRightCorner<3, 3>() =
      rotation * covariance.block<3, 3>(3, 3) * rotation.transpose();
  return out;
}

} // namespace fast_lio
