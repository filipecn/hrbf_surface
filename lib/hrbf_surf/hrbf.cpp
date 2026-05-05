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

/// \file   hrbf.cpp
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-05-02

#include <hrbf_surf/hrbf.h>

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace hrbf_surf {

Result<HRBFSystem2> HRBFSystem2::from(const std::vector<Point> &positions,
                                      const std::vector<Vector> &normals,
                                      const std::vector<h_index> &ids) {

  std::vector<Eigen::Matrix<Scalar, 3, 1>> points(ids.size());
  std::vector<Eigen::Matrix<Scalar, 3, 1>> _normals(ids.size());
  for (h_index i = 0; i < ids.size(); ++i) {
    for (h_index j = 0; j < 3; ++j) {
      points[i][j] = positions[ids[i]][j];
    }
    _normals[i][0] = normals[ids[i]].x;
    _normals[i][1] = normals[ids[i]].y;
    _normals[i][2] = normals[ids[i]].z;
  }

  HRBFSystem2 hrbf_s;

  hrbf_s.solver_.hermite_fit(points, _normals);

  return Result<HRBFSystem2>(std::move(hrbf_s));
}

Result<HRBFSystem2> HRBFSystem2::from(const std::vector<Point> &positions,
                                      const std::vector<Vector> &normals) {
  std::vector<h_index> ids;
  for (h_index i = 0; i < positions.size(); ++i)
    ids.emplace_back(i);
  return HRBFSystem2::from(positions, normals, ids);
}

Scalar HRBFSystem2::operator()(const Point &p) const {
  Eigen::Matrix<Scalar, 3, 1> v;
  v[0] = p.x;
  v[1] = p.y;
  v[2] = p.z;

  return solver_.eval(v);
}

} // namespace hrbf_surf
