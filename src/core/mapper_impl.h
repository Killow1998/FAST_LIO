#pragma once
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "runtime_log.h"
#include <fast_lio/mapper.h>
#include <ikd-Tree/ikd_Tree.h>
#include <pcl/filters/voxel_grid.h>

namespace fast_lio {
struct Mapper::Impl {
  Impl(const Config &value, OdometryObserver observer);
  bool sync(MeasureGroup &measurement);
  void enqueue(double stamp, PointCloudXYZI::Ptr cloud, bool livox);
  FrameOutput process();
  void pointBodyToWorld(const PointType *input, PointType *output);
  void lasermap_fov_segment();
  void map_incremental();
  void h_share_model(state_ikfom &state, esekfom::dyn_share_datastruct<double> &data);
  void saveCloud(bool final);
  Config config;
  OdometryObserver observer;
  Preprocess preprocess;
  ImuProcess imu;
  RuntimeLog runtime_log;
  struct Scan {
    double stamp;
    PointCloudXYZI::Ptr cloud;
  };
  std::deque<Scan> scans;
  std::deque<std::shared_ptr<const ImuSample>> imu_buffer;
  double last_timestamp_lidar = 0., last_timestamp_imu = -1.;
  double timediff_lidar_wrt_imu = 0., lidar_mean_scantime = 0.;
  bool timediff_set_flg = false, lidar_pushed = false, flg_first_scan = true;
  double first_lidar_time = 0., lidar_end_time = 0.;
  int scan_num = 0;
  MeasureGroup Measures;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state_point;
  V3D pos_lid = V3D::Zero();
  KD_TREE<PointType> ikdtree;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  PointCloudXYZI::Ptr feats_undistort{new PointCloudXYZI};
  PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI};
  PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI};
  PointCloudXYZI::Ptr normvec{new PointCloudXYZI};
  PointCloudXYZI::Ptr laserCloudOri{new PointCloudXYZI};
  PointCloudXYZI::Ptr corr_normvect{new PointCloudXYZI};
  PointCloudXYZI::Ptr saved_points{new PointCloudXYZI};
  std::vector<unsigned char> point_selected_surf;
  std::vector<PointVector> Nearest_Points;
  std::vector<BoxPointType> cub_needrm;
  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false, flg_EKF_inited = false;
  int feats_down_size = 0, effct_feat_num = 0;
  double match_time = 0., solve_time = 0.;
  int path_frames = 0, save_frames = 0, pcd_index = 0;
  std::string diagnostic;
};
} // namespace fast_lio
