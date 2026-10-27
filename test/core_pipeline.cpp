#include <cmath>
#include <fast_lio/mapper.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <unistd.h>

namespace {
struct Record {
  fast_lio::FrameStatus status;
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;
  size_t points;
};
pcl::PointCloud<pcl::PointXYZI> scene(double position, double offset) {
  pcl::PointCloud<pcl::PointXYZI> points;
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 9; ++j) {
      pcl::PointXYZI p;
      p.intensity = 10;
      p.x = offset + 1. + i * .2 - position;
      p.y = -1. + j * .2;
      p.z = -1.;
      points.push_back(p);
      p.x = offset + 3. - position;
      p.y = -1. + i * .2;
      p.z = -1. + j * .2;
      points.push_back(p);
      p.x = offset + 1. + i * .2 - position;
      p.y = 2.;
      p.z = -1. + j * .2;
      points.push_back(p);
    }
  return points;
}
std::vector<Record> feed(fast_lio::Mapper &mapper, bool moving, double offset = 0.,
                         fast_lio::Mapper *other = nullptr,
                         std::vector<Record> *other_records = nullptr) {
  std::vector<Record> records;
  for (int tick = 0; tick <= 700; ++tick) {
    const double time = int64_t(tick) * 5000000 / 1e9;
    fast_lio::ImuSample imu;
    imu.stamp_ns = int64_t(tick) * 5000000;
    imu.linear_acceleration.z() = 9.81;
    imu.linear_acceleration.x() = moving && time >= 1.5 && time < 2.5 ? .1 : 0.;
    EXPECT_TRUE(mapper.pushImu(imu));
    if (other) {
      auto stationary = imu;
      stationary.linear_acceleration.x() = 0.;
      EXPECT_TRUE(other->pushImu(stationary));
    }
    if (tick % 20 != 0)
      continue;
    const double elapsed = std::max(0., time - 1.5);
    const double position =
        moving ? (elapsed <= 1. ? .05 * elapsed * elapsed : .05 + .1 * (elapsed - 1.)) : 0.;
    auto points = scene(position, offset);
    pcl::PCLPointCloud2 cloud;
    pcl::toPCLPointCloud2(points, cloud);
    EXPECT_TRUE(mapper.pushPointCloud(time, cloud));
    auto output = mapper.process();
    records.push_back({output.status, output.odometry.position, output.odometry.orientation,
                       output.body_cloud ? output.body_cloud->size() : 0});
    if (other) {
      points = scene(0., 4.);
      pcl::toPCLPointCloud2(points, cloud);
      EXPECT_TRUE(other->pushPointCloud(time, cloud));
      const auto stationary = other->process();
      other_records->push_back({stationary.status, stationary.odometry.position,
                                stationary.odometry.orientation,
                                stationary.body_cloud ? stationary.body_cloud->size() : 0});
    }
  }
  return records;
}
fast_lio::Config config() {
  fast_lio::Config result;
  result.lidar_type = 4;
  result.extrinsic_est_en = false;
  result.filter_size_surf_min = .1;
  result.filter_size_map_min = .1;
  result.point_filter_num = 1;
  result.cube_len = 1000.;
  result.path_en = false;
  return result;
}
} // namespace
TEST(CorePipeline, MovingInstancesRemainIndependent) {
  std::vector<Record> baseline;
  {
    fast_lio::Mapper mapper(config());
    baseline = feed(mapper, true);
  }
  std::vector<Record> stationary_baseline;
  {
    fast_lio::Mapper mapper(config());
    stationary_baseline = feed(mapper, false, 4.);
  }
  fast_lio::Mapper moving(config()), stationary(config());
  std::vector<Record> stationary_interleaved;
  const auto interleaved = feed(moving, true, 0., &stationary, &stationary_interleaved);
  ASSERT_EQ(interleaved.size(), baseline.size());
  ASSERT_EQ(stationary_interleaved.size(), stationary_baseline.size());
  size_t mapped = 0;
  for (size_t i = 0; i < baseline.size(); ++i) {
    EXPECT_EQ(stationary_interleaved[i].status, stationary_baseline[i].status);
    EXPECT_EQ(stationary_interleaved[i].points, stationary_baseline[i].points);
    EXPECT_LT((stationary_interleaved[i].position - stationary_baseline[i].position).norm(), 1e-10);
    EXPECT_LT(stationary_interleaved[i].rotation.angularDistance(stationary_baseline[i].rotation),
              1e-10);
    EXPECT_EQ(interleaved[i].status, baseline[i].status);
    EXPECT_EQ(interleaved[i].points, baseline[i].points);
    EXPECT_LT((interleaved[i].position - baseline[i].position).norm(), 1e-10);
    EXPECT_LT(interleaved[i].rotation.angularDistance(baseline[i].rotation), 1e-10);
    if (baseline[i].status == fast_lio::FrameStatus::Mapped)
      ++mapped;
  }
  EXPECT_GT(mapped, 15u);
  EXPECT_GT(baseline.back().position.x(), .05);
}
TEST(CorePipeline, SaveAndBoundedLogSurviveRestart) {
  char pattern[] = "/tmp/fast-lio-core-XXXXXX";
  const char *made = mkdtemp(pattern);
  ASSERT_NE(made, nullptr);
  const std::filesystem::path directory(made);
  const auto log = directory / "runtime.log";
  {
    std::ofstream stream(log);
    stream << "start";
  }
  std::filesystem::resize_file(log, 99999980);
  setenv("FAST_LIO_LOG_DIR", directory.c_str(), 1);
  auto settings = config();
  settings.pcd_save_en = true;
  settings.pcd_directory = directory.string();
  settings.runtime_pos_log = true;
  for (int run = 0; run < 2; ++run) {
    fast_lio::Mapper mapper(settings);
    feed(mapper, true);
    mapper.finish();
    fast_lio::Cloud cloud;
    ASSERT_EQ(pcl::io::loadPCDFile((directory / "scans.pcd").string(), cloud), 0);
    EXPECT_GT(cloud.size(), 1000u);
  }
  EXPECT_LE(std::filesystem::file_size(log), 100000000u);
  unsetenv("FAST_LIO_LOG_DIR");
  std::filesystem::remove_all(directory);
}

TEST(CorePipeline, InvalidStartupSampleRestartsStationaryWindow) {
  fast_lio::Mapper mapper(config());
  pcl::PCLPointCloud2 cloud;
  pcl::toPCLPointCloud2(scene(0., 0.), cloud);
  double first_mapped = 0.;
  for (int tick = 0; tick <= 460; ++tick) {
    fast_lio::ImuSample sample;
    sample.stamp_ns = int64_t(tick) * 5000000;
    sample.linear_acceleration.z() = 9.81;
    if (tick == 160)
      sample.linear_acceleration.x() = std::numeric_limits<double>::quiet_NaN();
    ASSERT_TRUE(mapper.pushImu(sample));
    if (tick % 20 != 0)
      continue;
    ASSERT_TRUE(mapper.pushPointCloud(sample.seconds(), cloud));
    const auto output = mapper.process();
    if (output.status == fast_lio::FrameStatus::Mapped && first_mapped == 0.)
      first_mapped = output.scan_end;
  }
  EXPECT_GE(first_mapped, 1.9);
  EXPECT_LE(first_mapped, 2.3);
}
