#pragma once
#include <fast_lio/config.h>
#include <fast_lio/types.h>
#include <memory>
#include <pcl/PCLPointCloud2.h>

namespace fast_lio {
// One serial caller owns an instance. Returned clouds remain immutable.
class Mapper {
public:
  explicit Mapper(const Config &config, OdometryObserver observer = {});
  ~Mapper();
  Mapper(const Mapper &) = delete;
  Mapper &operator=(const Mapper &) = delete;
  bool pushImu(const ImuSample &sample);
  bool pushLivox(double stamp, const LivoxScan &scan);
  bool pushPointCloud(double stamp, const pcl::PCLPointCloud2 &scan);
  FrameOutput process();
  void finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fast_lio
