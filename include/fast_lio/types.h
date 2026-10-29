#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace fast_lio {
using Point = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<Point>;
struct ImuSample {
  int64_t stamp_ns = 0;
  double seconds() const { return stamp_ns / 1e9; }
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};
struct LivoxPoint {
  uint32_t offset_time = 0;
  float x = 0, y = 0, z = 0;
  uint8_t reflectivity = 0, tag = 0, line = 0;
};
struct LivoxScan {
  std::vector<LivoxPoint> points;
};
struct ImuPose {
  double offset_time = 0.;
  std::array<double, 3> acc{}, gyr{}, vel{}, pos{};
  std::array<double, 9> rot{};
};
struct Odometry {
  int64_t stamp_ns = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  // Both velocities refer to the IMU origin and are expressed in its body frame.
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  // Parent-frame pose covariance, ordered [position, rotation].
  std::array<double, 36> covariance{};
};
enum class OdometryKind { Imu, Lidar };
using OdometryObserver = std::function<void(OdometryKind, const Odometry &)>;

enum class FrameStatus {
  Waiting,
  FirstScan,
  Initializing,
  MapInitialized,
  InsufficientPoints,
  Mapped
};
struct FrameOutput {
  FrameStatus status = FrameStatus::Waiting;
  double scan_begin = 0., scan_end = 0.;
  Odometry odometry;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  Cloud::ConstPtr world_cloud, body_cloud;
  std::string diagnostic;
  bool append_path = false;
  bool consumed() const { return status != FrameStatus::Waiting; }
};
} // namespace fast_lio
