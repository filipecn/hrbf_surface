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

/// \file   point_cloud.cpp
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-04-30

#include <hrbf_surf/point_cloud.h>

namespace hrbf_surf {

PointCloud::Ptr PointCloud::from(const std::vector<Point> &positions,
                                 const std::vector<Vector> &normals) {
  auto pcl = PointCloud::Ptr::shared();
  pcl->positions_ = positions;
  pcl->normals_ = normals;
  pcl->pcl_ = std::make_unique<PCL>(
      Eigen::Map<PCL>(&pcl->positions_[0].x, pcl->positions_.size(), 3));
  pcl->tree_ = std::make_unique<KDTree>(3, std::cref(*(pcl->pcl_.get())), 10);
  return pcl;
}

Bounds PointCloud::computeBounds() const {
  Eigen::Vector3d min_pt = pcl_->colwise().minCoeff();
  Eigen::Vector3d max_pt = pcl_->colwise().maxCoeff();
  auto padding = 0.05; // Add a small margin for safety
  min_pt.array() -= padding;
  max_pt.array() += padding;
  return Bounds(Point(min_pt[0], min_pt[1], min_pt[2]),
                Point(max_pt[0], max_pt[1], max_pt[2]));
}

std::vector<h_index> PointCloud::searchBox(const Bounds &box) const {
  auto center = box.center();
  auto radius_sqr = box.diagonal().length2();
  std::vector<nanoflann::ResultItem<Eigen::Index, Scalar>> r_distances;
  nanoflann::SearchParameters params;
  auto n_found =
      tree_->index_->radiusSearch(&center[0], radius_sqr, r_distances, params);

  // filter points
  std::vector<h_index> indices;
  for (const auto &result : r_distances) {
    if (box.contains(positions_[result.first]))
      indices.emplace_back(result.first);
  }
  return indices;
}

const std::vector<Point> &PointCloud::positions() const { return positions_; }

const std::vector<Vector> &PointCloud::normals() const { return normals_; }

} // namespace hrbf_surf