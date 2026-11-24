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
#include <algorithm>
#include <omp.h>

namespace fast_lio {
void Mapper::Impl::h_share_model(state_ikfom &s,
                                 esekfom::dyn_share_datastruct<double> &ekfom_data) {
  double match_start = omp_get_wtime();
  const auto point_count = feats_down_body->size();

#if FAST_LIO_MATCH_THREADS > 1
#pragma omp parallel for num_threads(FAST_LIO_MATCH_THREADS)
#endif
  for (std::size_t i = 0; i < point_count; ++i) {
    PointType &point_body = feats_down_body->points[i];
    PointType &point_world = feats_down_world->points[i];

    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    auto &points_near = Nearest_Points[i];
    if (ekfom_data.converge) {
      vector<float> squared_distances(NUM_MATCH_POINTS);
      ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, squared_distances);
      point_selected_surf[i] = points_near.size() >= NUM_MATCH_POINTS &&
                               !(squared_distances.back() > 5);
    }

    if (!point_selected_surf[i])
      continue;

    Eigen::Vector4f pabcd;
    point_selected_surf[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f)) {
      float pd2 =
          pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s > 0.9) {
        point_selected_surf[i] = true;
        normvec->points[i].x = pabcd(0);
        normvec->points[i].y = pabcd(1);
        normvec->points[i].z = pabcd(2);
        normvec->points[i].intensity = pd2;
      }
    }
  }

  const auto matched_count = std::count(point_selected_surf.begin(),
                                        point_selected_surf.begin() + point_count, 1);
  if (matched_count == 0) {
    ekfom_data.valid = false;
    diagnostic = "No effective points";
    return;
  }

  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  ekfom_data.h_x = MatrixXd::Zero(matched_count, 12);
  ekfom_data.h.resize(matched_count);

  Eigen::Index row = 0;
  for (std::size_t i = 0; i < point_count; ++i) {
    if (!point_selected_surf[i]) continue;
    const PointType &laser_p = feats_down_body->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    const PointType &norm_p = normvec->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (config.extrinsic_est_en) {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
      ekfom_data.h_x.block<1, 12>(row, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
          VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(row, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0;
    }

    ekfom_data.h(row) = -norm_p.intensity;
    ++row;
  }
  solve_time += omp_get_wtime() - solve_start_;
}
} // namespace fast_lio
