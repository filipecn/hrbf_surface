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

/// \file   pou_surface.h
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-05-02

#pragma once

#include <hrbf_surf/hrbf.h>
#include <hrbf_surf/point_cloud.h>
#include <hrbf_surf/result.h>

namespace hrbf_surf {

class PoUSurface {
public:
  struct SurfaceMesh {
    Eigen::MatrixXd V; // Mesh Vertices
    Eigen::MatrixXi F; // Mesh Faces (triangles)
  };

  static Result<PoUSurface> from(const PointCloud &pcl, f64 cell_size,
                                 f64 overlap);
  static Result<PoUSurface> from(const PointCloud &pcl);

  f64 operator()(const hermes::geo::point3d &p) const;

  SurfaceMesh mesh(f64 voxel_size) const;
  SurfaceMesh partitionMesh(h_index index, f64 voxel_size) const;

private:
  struct PartitionData {
    hermes::geo::point3d center;
    std::vector<h_index> indices;
    HRBFSystem hrbf;
    f64 radius;
  };

  hermes::geo::bounds::BoundingBox3<f64> bounds_;
  std::vector<PartitionData> partitions_;
};

} // namespace hrbf_surf