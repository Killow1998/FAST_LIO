#include "mapper_impl.h"
#include <cmath>

namespace fast_lio {
bool Mapper::pushImu(const ImuSample &raw) {
  if (raw.stamp_ns < 0)
    return false;
  auto &state = *impl_;
  auto sample = std::make_shared<ImuSample>(raw);
  double shifted = raw.seconds() - state.config.time_diff_lidar_to_imu;
  if (std::abs(state.timediff_lidar_wrt_imu) > .1 && state.config.time_sync_en)
    shifted = state.timediff_lidar_wrt_imu + raw.seconds();
  sample->stamp_ns = static_cast<int64_t>(shifted * 1e9);
  if (sample->seconds() < state.last_timestamp_imu) {
    state.imu_buffer.clear();
    state.diagnostic = "IMU timestamp rolled back; input buffer cleared";
  }
  state.last_timestamp_imu = sample->seconds();
  state.imu_buffer.push_back(std::move(sample));
  return true;
}

bool Mapper::pushLivox(double stamp, const LivoxScan &scan) {
  if (!std::isfinite(stamp) || stamp < 0.)
    return false;
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  impl_->preprocess.process(scan, *cloud);
  impl_->enqueue(stamp, std::move(cloud), true);
  return true;
}

bool Mapper::pushPointCloud(double stamp, const pcl::PCLPointCloud2 &scan) {
  if (!std::isfinite(stamp) || stamp < 0. || scan.point_step == 0 ||
      uint64_t(scan.width) * scan.point_step > scan.row_step ||
      uint64_t(scan.row_step) * scan.height > scan.data.size())
    return false;
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  impl_->preprocess.process(scan, *cloud);
  impl_->enqueue(stamp, std::move(cloud), false);
  return true;
}

void Mapper::Impl::enqueue(double stamp, PointCloudXYZI::Ptr cloud,
                           bool livox) {
  if (stamp < last_timestamp_lidar) {
    scans.clear();
    lidar_pushed = false;
    diagnostic = "LiDAR timestamp rolled back; paired scan buffer cleared";
  }
  last_timestamp_lidar = stamp;
  if (livox && config.time_sync_en && !timediff_set_flg &&
      std::abs(stamp - last_timestamp_imu) > 1. && !imu_buffer.empty()) {
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = stamp + .1 - last_timestamp_imu;
    diagnostic = "LiDAR/IMU self synchronization applied";
  }
  scans.push_back({stamp, std::move(cloud)});
}

bool Mapper::Impl::sync(MeasureGroup &measurement) {
  if (scans.empty() || imu_buffer.empty())
    return false;
  if (!lidar_pushed) {
    measurement.lidar = scans.front().cloud;
    measurement.lidar_beg_time = scans.front().stamp;
    if (measurement.lidar->size() <= 1 ||
        measurement.lidar->back().curvature / double(1000) <
            .5 * lidar_mean_scantime) {
      lidar_end_time = measurement.lidar_beg_time + lidar_mean_scantime;
    } else {
      ++scan_num;
      lidar_end_time = measurement.lidar_beg_time +
                       measurement.lidar->back().curvature / double(1000);
      lidar_mean_scantime +=
          (measurement.lidar->back().curvature / double(1000) -
           lidar_mean_scantime) /
          scan_num;
    }
    if (config.lidar_type == MARSIM)
      lidar_end_time = measurement.lidar_beg_time;
    measurement.lidar_end_time = lidar_end_time;
    lidar_pushed = true;
  }
  if (last_timestamp_imu < lidar_end_time)
    return false;
  measurement.imu.clear();
  while (!imu_buffer.empty() &&
         imu_buffer.front()->seconds() <= lidar_end_time) {
    measurement.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }
  scans.pop_front();
  lidar_pushed = false;
  if (measurement.imu.empty()) {
    diagnostic = "Dropped LiDAR scan without an IMU sample before its end";
    return false;
  }
  return true;
}
} // namespace fast_lio
