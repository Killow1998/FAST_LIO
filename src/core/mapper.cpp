// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "mapper_impl.h"
#include <filesystem>
#include <iomanip>
#include <omp.h>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <stdexcept>

namespace fast_lio {
Mapper::Impl::Impl(const Config &value, OdometryObserver callback)
    : config(value), observer(std::move(callback)), runtime_log(value.runtime_pos_log) {
  preprocess.set(config.feature_enabled, config.lidar_type, config.blind, config.point_filter_num);
  preprocess.N_SCANS = config.scan_lines;
  preprocess.SCAN_RATE = config.scan_rate;
  preprocess.time_unit = config.time_unit;
  downSizeFilterSurf.setLeafSize(config.filter_size_surf_min, config.filter_size_surf_min,
                                 config.filter_size_surf_min);
  V3D translation;
  translation << VEC_FROM_ARRAY(config.extrinT);
  M3D rotation;
  rotation << MAT_FROM_ARRAY(config.extrinR);
  imu.set_extrinsic(translation, rotation);
  imu.set_gyr_cov(V3D::Constant(config.gyr_cov));
  imu.set_acc_cov(V3D::Constant(config.acc_cov));
  imu.set_gyr_bias_cov(V3D::Constant(config.b_gyr_cov));
  imu.set_acc_bias_cov(V3D::Constant(config.b_acc_cov));
  imu.setGravity(config.gravity_m_s2);
  imu.lidar_type = config.lidar_type;
  double epsilon[23];
  std::fill(epsilon, epsilon + 23, .001);
  kf.init_dyn_share(
      get_f, df_dx, df_dw,
      [this](state_ikfom &state, esekfom::dyn_share_datastruct<double> &data) {
        h_share_model(state, data);
      },
      config.NUM_MAX_ITERATIONS, epsilon);
}
Mapper::Mapper(const Config &c, OdometryObserver observer) {
  if (c.extrinT.size() != 3 || c.extrinR.size() != 9 || c.scan_lines < 1 || c.scan_lines > 128 ||
      c.point_filter_num < 1 || c.scan_rate < 1 || c.NUM_MAX_ITERATIONS < 1 || c.lidar_type < 1 ||
      c.lidar_type > 4 || c.time_unit < 0 || c.time_unit > 3 || !(c.gravity_m_s2 > 0.) ||
      !(c.filter_size_surf_min > 0.) || !(c.filter_size_map_min > 0.) || !(c.cube_len > 0.))
    throw std::invalid_argument("Invalid FAST-LIO configuration: extrinsics, "
                                "preprocessing or filter limits");
  impl_ = std::make_unique<Impl>(c, std::move(observer));
}
Mapper::~Mapper() = default;
FrameOutput Mapper::process() { return impl_->process(); }
void Mapper::finish() { impl_->saveCloud(true); }

FrameOutput Mapper::Impl::process() {
  FrameOutput out;
  if (!sync(Measures))
    return out;
  out.scan_begin = Measures.lidar_beg_time;
  out.scan_end = Measures.lidar_end_time;
  out.diagnostic.swap(diagnostic);
  if (flg_first_scan) {
    first_lidar_time = Measures.lidar_beg_time;
    imu.first_lidar_time = first_lidar_time;
    flg_first_scan = false;
    out.status = FrameStatus::FirstScan;
    return out;
  }
  const double started = omp_get_wtime();
  match_time = solve_time = 0.;
  const bool initializing = imu.initializing();
  imu.Process(Measures, kf, feats_undistort, observer);
  if (!imu.report.empty())
    out.diagnostic = imu.report;
  state_point = kf.get_x();
  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
  if (feats_undistort->empty()) {
    out.status = initializing ? FrameStatus::Initializing : FrameStatus::InsufficientPoints;
    return out;
  }
  flg_EKF_inited = Measures.lidar_beg_time - first_lidar_time >= 1.;
  lasermap_fov_segment();
  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  feats_down_size = feats_down_body->size();
  if (ikdtree.Root_Node == nullptr) {
    if (feats_down_size > 5) {
      ikdtree.set_downsample_param(config.filter_size_map_min);
      feats_down_world->resize(feats_down_size);
      for (int i = 0; i < feats_down_size; ++i)
        pointBodyToWorld(&feats_down_body->points[i], &feats_down_world->points[i]);
      ikdtree.Build(feats_down_world->points);
    }
    out.status = ikdtree.Root_Node ? FrameStatus::MapInitialized : FrameStatus::InsufficientPoints;
    return out;
  }
  if (feats_down_size < 5) {
    out.status = FrameStatus::InsufficientPoints;
    return out;
  }
  normvec->resize(feats_down_size);
  feats_down_world->resize(feats_down_size);
  if (point_selected_surf.size() < size_t(feats_down_size))
    point_selected_surf.resize(feats_down_size, 1);
  if (res_last.size() < size_t(feats_down_size))
    res_last.resize(feats_down_size, -1000.f);
  Nearest_Points.resize(feats_down_size);
  double solve_H_time = 0.;
  kf.update_iterated_dyn_share_modified(.001, solve_H_time);
  state_point = kf.get_x();
  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
  out.odometry.stamp_ns = static_cast<int64_t>(lidar_end_time * 1e9);
  out.odometry.position = state_point.pos;
  out.odometry.orientation = Eigen::Quaterniond(state_point.rot);
  out.odometry.velocity = state_point.vel;
  out.odometry.angular_velocity = Measures.imu.back()->angular_velocity;
  // Preserve the existing published covariance sequence in this structural
  // change.
  out.odometry.covariance = published_covariance;
  if (observer)
    observer(OdometryKind::Lidar, out.odometry);
  const auto covariance = kf.get_P();
  for (int i = 0; i < 6; ++i) {
    const int k = i < 3 ? i + 3 : i - 3;
    for (int j = 0; j < 6; ++j)
      published_covariance[i * 6 + j] = covariance(k, j < 3 ? j + 3 : j - 3);
  }
  out.gravity = state_point.grav.vec;
  map_incremental();
  if (config.scan_pub_en || config.pcd_save_en) {
    const auto source =
        config.dense_pub_en || !config.scan_pub_en ? feats_undistort : feats_down_body;
    PointCloudXYZI::Ptr world(new PointCloudXYZI(source->size(), 1));
    for (size_t i = 0; i < source->size(); ++i)
      pointBodyToWorld(&source->points[i], &world->points[i]);
    if (config.scan_pub_en)
      out.world_cloud = world;
    if (config.pcd_save_en) {
      if (source == feats_undistort)
        *saved_points += *world;
      else {
        for (const auto &point : feats_undistort->points) {
          PointType transformed;
          pointBodyToWorld(&point, &transformed);
          saved_points->push_back(transformed);
        }
      }
      ++save_frames;
      saveCloud(false);
    }
  }
  if (config.scan_pub_en && config.scan_body_pub_en) {
    PointCloudXYZI::Ptr body(new PointCloudXYZI(feats_undistort->size(), 1));
    for (size_t i = 0; i < body->size(); ++i) {
      const auto &p = feats_undistort->points[i];
      const V3D transformed(state_point.offset_R_L_I * V3D(p.x, p.y, p.z) +
                            state_point.offset_T_L_I);
      auto &q = body->points[i];
      q.x = transformed.x();
      q.y = transformed.y();
      q.z = transformed.z();
      q.intensity = p.intensity;
    }
    out.body_cloud = body;
  }
  out.append_path = config.path_en && (++path_frames % 10 == 0);
  out.status = FrameStatus::Mapped;
  if (!diagnostic.empty())
    out.diagnostic.swap(diagnostic);
  if (config.runtime_pos_log) {
    std::ostringstream row;
    row << std::setprecision(16) << lidar_end_time << ',' << state_point.pos.transpose() << ','
        << state_point.rot.coeffs().transpose() << ',' << (omp_get_wtime() - started) << ','
        << match_time << ',' << solve_time << ',' << feats_down_size << ',' << ikdtree.size();
    runtime_log.write(row.str());
  }
  return out;
}

void Mapper::Impl::saveCloud(bool final) {
  if (!config.pcd_save_en || saved_points->empty() ||
      (!final && (config.pcd_save_interval <= 0 || save_frames < config.pcd_save_interval)))
    return;
  const std::filesystem::path base = config.pcd_directory.empty()
                                         ? std::filesystem::path(ROOT_DIR) / "PCD"
                                         : std::filesystem::path(config.pcd_directory);
  std::filesystem::create_directories(base);
  const auto file = base / (final ? "scans.pcd" : "scans_" + std::to_string(++pcd_index) + ".pcd");
  if (pcl::io::savePCDFileBinary(file.string(), *saved_points) != 0)
    throw std::runtime_error("Could not save FAST-LIO point cloud: " + file.string());
  saved_points->clear();
  save_frames = 0;
}
} // namespace fast_lio
