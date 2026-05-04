/* Copyright (c) 2026, FilipeCN.
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/// \file   pou_surface.cpp
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-05-02

#include <hrbf_surf/pou_surface.h>

#include <igl/copyleft/marching_cubes.h>
#include <omp.h>

namespace hrbf_surf {

Result<PoUSurface> PoUSurface::from(const PointCloud &pcl, f64 cell_size,
                                    f64 overlap) {
  auto bounds = pcl.computeBounds();

  // define a overlap for PoU
  f64 half_cell_size = cell_size / 2.0;
  hermes::geo::vec3d half_search_box(half_cell_size + cell_size * overlap);

  auto grid_size_f = (bounds.upper - bounds.lower) / cell_size;
  hermes::size3 grid_size(static_cast<h_index>(std::ceil(grid_size_f.x)) + 1,
                          static_cast<h_index>(std::ceil(grid_size_f.y)) + 1,
                          static_cast<h_index>(std::ceil(grid_size_f.z)) + 1);

  HERMES_INFO("grid setup (total partitions = {})", grid_size.total());
  // setup partitions
  PoUSurface ps;
  ps.bounds_ = bounds;
  for (auto ijk : hermes::range3(grid_size)) {
    PartitionData pd;
    pd.center = hermes::geo::point3d(ijk.i * cell_size + half_cell_size,
                                     ijk.j * cell_size + half_cell_size,
                                     ijk.k * cell_size + half_cell_size);
    hermes::geo::bounds::BoundingBox3<f64> search_box(
        pd.center - half_search_box, pd.center + half_search_box);
    pd.indices = pcl.searchBox(search_box);
    pd.radius = half_search_box.x;
    // consider only cells that provide enough points for a stable HRBF
    if (pd.indices.size() > 15) {
      ps.partitions_.emplace_back(pd);
    }
  }

  HERMES_INFO("solving for {} partitions", ps.partitions_.size());
  if (ps.partitions_.empty())
    return HsResult::inputError();

  // compute parititions
#pragma omp parallel for schedule(dynamic)
  for (h_index i = 0; i < ps.partitions_.size(); ++i) {
    auto hrbf = HRBFSystem::from(pcl.positions(), pcl.normals(),
                                 ps.partitions_[i].indices);
    ps.partitions_[i].hrbf = std::move(*hrbf);
  }

  return Result<PoUSurface>(std::move(ps));
}

Result<PoUSurface> PoUSurface::from(const PointCloud &pcl) {
  auto bounds = pcl.computeBounds();
  PoUSurface ps;
  ps.bounds_ = bounds;
  PartitionData pd;
  pd.center = bounds.center();
  pd.indices = pcl.searchBox(bounds);
  pd.radius = bounds.diagonal().length() / 2.0;
  // consider only cells that provide enough points for a stable HRBF
  if (pd.indices.size() > 15) {
    ps.partitions_.emplace_back(pd);
    auto hrbf = HRBFSystem::from(pcl.positions(), pcl.normals(),
                                 ps.partitions_[0].indices);
    ps.partitions_[0].hrbf = std::move(*hrbf);
  }
  return Result<PoUSurface>(std::move(ps));
}

f64 wendlandWeight(f64 distance, f64 radius) {
  f64 q = distance / radius;
  if (q >= 1.0)
    return 0.0;
  f64 t = 1.0 - q;
  return (t * t * t * t) * (4.0 * q + 1.0); // Wendland C2
}

f64 PoUSurface::operator()(const hermes::geo::point3d &p) const {
  f64 total_value = 0.0;
  f64 total_weight = 0.0;

  for (const auto &partition : partitions_) {
    f64 dist = (p - partition.center).length();
    if (dist < partition.radius) {
      f64 w = wendlandWeight(dist, partition.radius);
      total_value += w * partition.hrbf(p);
      total_weight += w;
    }
  }

  if (total_weight < 1e-9)
    return 1.0; // Outside of influence
  return total_value / total_weight;
}

PoUSurface::SurfaceMesh PoUSurface::mesh(f64 voxel_size) const {
  hermes::size3 resolution(
      std::ceil((bounds_.upper.x - bounds_.lower.x) / voxel_size) + 1,
      std::ceil((bounds_.upper.y - bounds_.lower.y) / voxel_size) + 1,
      std::ceil((bounds_.upper.z - bounds_.lower.z) / voxel_size) + 1);
  auto N = resolution.total();

  Eigen::MatrixXd GV(N, 3); // Grid Vertex positions
  Eigen::VectorXd S(N);     // Scalar values

#pragma omp parallel for collapse(3)
  for (h_index z = 0; z < resolution.depth; ++z) {
    for (h_index y = 0; y < resolution.height; ++y) {
      for (h_index x = 0; x < resolution.width; ++x) {
        auto idx =
            x + y * resolution.width + z * resolution.width * resolution.height;

        hermes::geo::point3d p(bounds_.lower.x + x * voxel_size,
                               bounds_.lower.y + y * voxel_size,
                               bounds_.lower.z + z * voxel_size);
        GV.row(idx)[0] = p.x;
        GV.row(idx)[1] = p.y;
        GV.row(idx)[2] = p.z;

        S(idx) = (*this)(p);
      }
    }
  }

  PoUSurface::SurfaceMesh m;
  // Isovalue for HRBF is 0.0
  igl::copyleft::marching_cubes(S, GV, resolution.width, resolution.height,
                                resolution.depth, 0.0, m.V, m.F);
  return m;
}

PoUSurface::SurfaceMesh PoUSurface::partitionMesh(h_index index,
                                                  f64 voxel_size) const {
  auto radius = hermes::geo::vec3d(partitions_[index].radius);
  auto lower = partitions_[index].center - radius;
  auto upper = partitions_[index].center + radius;

  hermes::size3 resolution(std::ceil((upper.x - lower.x) / voxel_size) + 1,
                           std::ceil((upper.y - lower.y) / voxel_size) + 1,
                           std::ceil((upper.z - lower.z) / voxel_size) + 1);
  auto N = resolution.total();

  Eigen::MatrixXd GV(N, 3); // Grid Vertex positions
  Eigen::VectorXd S(N);     // Scalar values

#pragma omp parallel for collapse(3)
  for (h_index z = 0; z < resolution.depth; ++z) {
    for (h_index y = 0; y < resolution.height; ++y) {
      for (h_index x = 0; x < resolution.width; ++x) {
        auto idx =
            x + y * resolution.width + z * resolution.width * resolution.height;

        hermes::geo::point3d p(lower.x + x * voxel_size,
                               lower.y + y * voxel_size,
                               lower.z + z * voxel_size);
        GV.row(idx)[0] = p.x;
        GV.row(idx)[1] = p.y;
        GV.row(idx)[2] = p.z;

        S(idx) = partitions_[index].hrbf(p);
      }
    }
  }

  PoUSurface::SurfaceMesh m;
  // Isovalue for HRBF is 0.0
  igl::copyleft::marching_cubes(S, GV, resolution.width, resolution.height,
                                resolution.depth, 0.0, m.V, m.F);
  return m;
}

} // namespace hrbf_surf
