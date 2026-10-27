#pragma once
#include <string>
#include <vector>

namespace fast_lio {
struct Config {
  bool path_en = true;
  bool scan_pub_en = true;
  bool dense_pub_en = true;
  bool scan_body_pub_en = true;
  int NUM_MAX_ITERATIONS = 4;
  std::string pcd_directory = "";
  bool time_sync_en = false;
  double time_diff_lidar_to_imu = 0.0;
  double filter_size_surf_min = 0.5;
  double filter_size_map_min = 0.5;
  double cube_len = 200.0;
  float DET_RANGE = 300.f;
  double gyr_cov = 0.1;
  double acc_cov = 0.1;
  double b_gyr_cov = 0.0001;
  double b_acc_cov = 0.0001;
  double blind = 0.01;
  int lidar_type = 1;
  int scan_lines = 16;
  int time_unit = 2;
  int scan_rate = 10;
  int point_filter_num = 2;
  bool feature_enabled = false;
  bool runtime_pos_log = false;
  bool extrinsic_est_en = true;
  bool pcd_save_en = false;
  int pcd_save_interval = -1;
  std::vector<double> extrinT = {0., 0., 0.};
  std::vector<double> extrinR = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
  double gravity_m_s2 = 9.81;
};

} // namespace fast_lio
