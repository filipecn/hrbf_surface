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

Result<PoUSurface::Ptr> PoUSurface::from(PointCloud ::Ptr pcl, Scalar cell_size,
                                         Scalar overlap) {
  auto bounds = pcl->computeBounds();

  // define a overlap for PoU
  Scalar half_cell_size = cell_size / 2.0;
  Vector half_search_box(half_cell_size + cell_size * overlap);

  auto grid_size_f = (bounds.upper - bounds.lower) / cell_size;
  hermes::size3 grid_size(static_cast<h_index>(std::ceil(grid_size_f.x)) + 1,
                          static_cast<h_index>(std::ceil(grid_size_f.y)) + 1,
                          static_cast<h_index>(std::ceil(grid_size_f.z)) + 1);

  HERMES_INFO("grid setup (total partitions = {})", grid_size.total());
  // setup partitions
  auto ps = PoUSurface::Ptr::shared();
  ps->pcl_ = pcl;
  ps->bounds_ = bounds;
  HERMES_LOG_VARIABLE(bounds);
  HERMES_LOG_VARIABLE(grid_size);
  HERMES_LOG_VARIABLE(cell_size);
  HERMES_LOG_VARIABLE(grid_size_f);
  HERMES_LOG_VARIABLE(half_cell_size);
  for (auto ijk : hermes::range3(grid_size)) {
    PartitionData pd;
    auto center = Point(bounds.lower.x + ijk.i * cell_size + half_cell_size,
                        bounds.lower.y + ijk.j * cell_size + half_cell_size,
                        bounds.lower.z + ijk.k * cell_size + half_cell_size);
    Bounds search_box(center - half_search_box, center + half_search_box);
    pd.indices = pcl->searchBox(search_box);
    pd.bounds = search_box;
    // consider only cells that provide enough points for a stable HRBF
    if (pd.indices.size() > 15) {
      ps->partitions_.emplace_back(pd);
    }
  }

  HERMES_INFO("solving for {} partitions", ps->partitions_.size());
  if (ps->partitions_.empty())
    return HsResult::inputError();

  // compute parititions
#pragma omp parallel for schedule(dynamic)
  for (h_index i = 0; i < ps->partitions_.size(); ++i) {
    ps->partitions_[i].rbf.init(pcl->positions(), ps->partitions_[i].indices);
    ps->partitions_[i].hrbf.init(pcl->positions(), pcl->normals(),
                                 ps->partitions_[i].indices);
    if (ps->partitions_[i].hrbf.hasNaN()) {
      HERMES_ERROR("invalid hrbf system");
    }
    if (ps->partitions_[i].rbf.hasNaN()) {
      HERMES_ERROR("invalid hrbf system");
    }
  }
  HERMES_INFO("finished solving partitions");

  return Result<PoUSurface::Ptr>(std::move(ps));
}

Result<PoUSurface::Ptr> PoUSurface::from(PointCloud::Ptr pcl) {
  auto bounds = pcl->computeBounds();

  auto ps = PoUSurface::Ptr::shared();
  ps->bounds_ = bounds;
  ps->pcl_ = pcl;

  PartitionData pd;
  pd.bounds = bounds;
  pd.indices = pcl->searchBox(bounds);

  // consider only cells that provide enough points for a stable HRBF
  if (pd.indices.size() > 15) {
    ps->partitions_.emplace_back(pd);
    HERMES_INFO("computing single hrbf with {} indices and bounds {}",
                ps->partitions_[0].indices.size(), hermes::to_string(bounds));
    ps->partitions_[0].rbf.init(pcl->positions(), ps->partitions_[0].indices);
    ps->partitions_[0].hrbf.init(pcl->positions(), pcl->normals(),
                                 ps->partitions_[0].indices);
    HERMES_LOG_VARIABLE(ps->partitions_[0].indices.size());
    HERMES_LOG_VARIABLE(ps->partitions_[0].hrbf.hasNaN());
    HERMES_LOG_VARIABLE(ps->partitions_[0].rbf.hasNaN());
  }
  return Result<PoUSurface::Ptr>(std::move(ps));
}

Scalar wendlandWeight(Scalar distance, Scalar radius) {
  Scalar q = distance / radius;
  if (q >= 1.0)
    return 0.0;
  Scalar t = 1.0 - q;
  return (t * t * t * t) * (4.0 * q + 1.0); // Wendland C2
}

Scalar PoUSurface::operator()(const Point &p) const {

  // find partitions this point belongs
  h_index partition_count = 0;
  Scalar sum = 0.0;
  for (const auto &partition : partitions_) {
    if (partition.bounds.contains(p)) {
      sum += partition.hrbf(pcl_->positions(), partition.indices, p);
      // sum += partition.rbf(pcl_->positions(), partition.indices, p);
    }
  }
  if (partition_count)
    return sum / partition_count;

  // get closest partition
  h_index closest_idx = 0;
  Scalar closest_dist = 1 << 20;
  for (h_index i = 0; i < partitions_.size(); ++i) {
    auto dist = hermes::geo::distance2(p, partitions_[i].bounds.center());
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  // return partitions_[closest_idx].rbf(pcl_->positions(),
  //                                     partitions_[closest_idx].indices, p);
  // compute from closest partition
  return partitions_[closest_idx].hrbf(pcl_->positions(),
                                       partitions_[closest_idx].indices, p);

  Scalar total_value = 0.0;
  Scalar total_weight = 0.0;

  for (const auto &partition : partitions_) {
    Scalar dist = (p - partition.bounds.center()).length();
    // if (dist < partition.radius * 2.0) {
    Scalar w = wendlandWeight(dist, partition.bounds.diagonal().length() / 2.0);
    total_value += w * partition.hrbf(pcl_->positions(), partition.indices, p);
    total_weight += w;
    // }
  }

  if (total_weight < 1e-9)
    return 1.0; // Outside of influence
  return total_value / total_weight;
}

PoUSurface::SurfaceMesh PoUSurface::mesh(Scalar voxel_size) const {
  hermes::size3 resolution(
      std::ceil((bounds_.upper.x - bounds_.lower.x) / voxel_size) + 1,
      std::ceil((bounds_.upper.y - bounds_.lower.y) / voxel_size) + 1,
      std::ceil((bounds_.upper.z - bounds_.lower.z) / voxel_size) + 1);
  auto N = resolution.total();

  Eigen::MatrixXd GV(N, 3); // Grid Vertex positions
  Eigen::VectorXd S(N);     // Scalar values

  HERMES_LOG_VARIABLE(bounds_);
  HERMES_LOG_VARIABLE(voxel_size);
  HERMES_LOG_VARIABLE(resolution);
  HERMES_INFO("evaluating surface");
#pragma omp parallel for collapse(3)
  for (h_index z = 0; z < resolution.depth; ++z) {
    for (h_index y = 0; y < resolution.height; ++y) {
      for (h_index x = 0; x < resolution.width; ++x) {
        auto idx =
            x + y * resolution.width + z * resolution.width * resolution.height;

        Point p(bounds_.lower.x + x * voxel_size,
                bounds_.lower.y + y * voxel_size,
                bounds_.lower.z + z * voxel_size);
        GV.row(idx)[0] = p.x;
        GV.row(idx)[1] = p.y;
        GV.row(idx)[2] = p.z;

        S(idx) = (*this)(p);
      }
    }
  }

  HERMES_INFO("running marching cubes");
  PoUSurface::SurfaceMesh m;
  // Isovalue for HRBF is 0.0
  igl::copyleft::marching_cubes(S, GV, resolution.width, resolution.height,
                                resolution.depth, 0.0, m.V, m.F);
  return m;
}

PoUSurface::SurfaceMesh PoUSurface::partitionMesh(h_index index,
                                                  Scalar voxel_size) const {
  const auto diagonal = partitions_[index].bounds.diagonal() / voxel_size;
  const auto lower = partitions_[index].bounds.lower;
  const hermes::size3 resolution(std::ceil(diagonal.x) + 1,
                                 std::ceil(diagonal.y) + 1,
                                 std::ceil(diagonal.z) + 1);
  auto N = resolution.total();

  HERMES_LOG_VARIABLE(resolution);

  Eigen::MatrixXd GV(N, 3); // Grid Vertex positions
  Eigen::VectorXd S(N);     // Scalar values

  HERMES_INFO("evaluating surface");
#pragma omp parallel for collapse(3)
  for (h_index z = 0; z < resolution.depth; ++z) {
    for (h_index y = 0; y < resolution.height; ++y) {
      for (h_index x = 0; x < resolution.width; ++x) {
        auto idx =
            x + y * resolution.width + z * resolution.width * resolution.height;

        Point p(lower.x + x * voxel_size, lower.y + y * voxel_size,
                lower.z + z * voxel_size);
        GV.row(idx)[0] = p.x;
        GV.row(idx)[1] = p.y;
        GV.row(idx)[2] = p.z;
        // consider only points inside
        S(idx) = 1.0;
        // if (partitions_[index].bounds.contains(p))
        S(idx) = partitions_[index].rbf(pcl_->positions(),
                                        partitions_[index].indices, p);
        // S(idx) = partitions_[index].hrbf(pcl_->positions(),
        //                                  partitions_[index].indices, p);
      }
    }
  }

  HERMES_INFO("running marching cubes");
  PoUSurface::SurfaceMesh m;
  // Isovalue for HRBF is 0.0
  igl::copyleft::marching_cubes(S, GV, resolution.width, resolution.height,
                                resolution.depth, 0.0, m.V, m.F);
  return m;
}

const PoUSurface::PartitionData &PoUSurface::partition(h_index i) const {
  return partitions_[i];
}

} // namespace hrbf_surf
