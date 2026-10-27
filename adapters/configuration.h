#pragma once
#include <fast_lio/config.h>

namespace fast_lio::adapter {
struct NodeConfig {
  fast_lio::Config core;
  std::string lid_topic = "/livox/lidar";
  std::string imu_topic = "/livox/imu";
  std::string gravity_topic = "/lio_gravity";
  std::string replay_lidar_ack_topic = "";
  std::string world_frame = "camera_init";
  std::string body_frame = "body";
};

// Both adapters bind this single parameter catalog to their native API.
template <class Reader> NodeConfig readConfig(Reader &&read) {
  NodeConfig node;
  auto &config = node.core;
  // Retain declared ROS parameters that the original estimator did not use.
  std::string unused_map_path;
  double unused_corner_leaf = .5, unused_fov = 180.;
  read("publish.path_en", config.path_en);
  read("publish.scan_publish_en", config.scan_pub_en);
  read("publish.dense_publish_en", config.dense_pub_en);
  read("publish.scan_bodyframe_pub_en", config.scan_body_pub_en);
  read("max_iteration", config.NUM_MAX_ITERATIONS);
  read("map_file_path", unused_map_path);
  read("common.lid_topic", node.lid_topic);
  read("common.imu_topic", node.imu_topic);
  read("common.gravity_topic", node.gravity_topic);
  read("common.replay_lidar_ack_topic", node.replay_lidar_ack_topic);
  read("common.time_sync_en", config.time_sync_en);
  read("common.time_offset_lidar_to_imu", config.time_diff_lidar_to_imu);
  read("filter_size_corner", unused_corner_leaf);
  read("filter_size_surf", config.filter_size_surf_min);
  read("filter_size_map", config.filter_size_map_min);
  read("cube_side_length", config.cube_len);
  read("mapping.det_range", config.DET_RANGE);
  read("mapping.fov_degree", unused_fov);
  read("mapping.gyr_cov", config.gyr_cov);
  read("mapping.acc_cov", config.acc_cov);
  read("mapping.b_gyr_cov", config.b_gyr_cov);
  read("mapping.b_acc_cov", config.b_acc_cov);
  read("preprocess.blind", config.blind);
  read("preprocess.lidar_type", config.lidar_type);
  read("preprocess.scan_line", config.scan_lines);
  read("preprocess.timestamp_unit", config.time_unit);
  read("preprocess.scan_rate", config.scan_rate);
  read("point_filter_num", config.point_filter_num);
  read("feature_extract_enable", config.feature_enabled);
  read("runtime_pos_log_enable", config.runtime_pos_log);
  read("mapping.extrinsic_est_en", config.extrinsic_est_en);
  read("pcd_save.pcd_save_en", config.pcd_save_en);
  read("pcd_save.interval", config.pcd_save_interval);
  read("mapping.extrinsic_T", config.extrinT);
  read("mapping.extrinsic_R", config.extrinR);
  read("common.world_frame", node.world_frame);
  read("common.body_frame", node.body_frame);
  read("common.gravity_m_s2", config.gravity_m_s2);
  return node;
}
} // namespace fast_lio::adapter
