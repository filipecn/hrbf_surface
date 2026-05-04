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

/// \file   point_cloud.h
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-04-30

#pragma once

#include <Eigen/Dense>
#include <nanoflann.hpp>

#include <hermes/geometry/bounds.h>
#include <hermes/geometry/normal.h>
#include <hermes/geometry/point.h>

namespace hrbf_surf {

using PCL = Eigen::Matrix<f64, Eigen::Dynamic, 3, Eigen::RowMajor>;
using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<PCL>;

class PointCloud {
public:
  static PointCloud from(const std::vector<hermes::geo::point3d> &positions,
                         const std::vector<hermes::geo::normal3d> &normals);

  hermes::geo::bounds::BoundingBox3<f64> computeBounds() const;
  std::vector<h_index>
  searchBox(const hermes::geo::bounds::BoundingBox3<f64> &box) const;

  const std::vector<hermes::geo::point3d> &positions() const;
  const std::vector<hermes::geo::normal3d> &normals() const;

private:
  std::unique_ptr<KDTree> tree_;
  std::unique_ptr<PCL> pcl_;
  std::vector<hermes::geo::point3d> positions_;
  std::vector<hermes::geo::normal3d> normals_;
};

} // namespace hrbf_surf