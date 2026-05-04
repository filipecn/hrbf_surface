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

/// \file   hrbf.h
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-04-30

#pragma once

#include <hrbf_surf/hrbf_core.h>
#include <hrbf_surf/hrbf_phi_funcs.h>
#include <hrbf_surf/result.h>

#include <Eigen/Dense>
#include <vector>

#include <hermes/geometry/normal.h>
#include <hermes/geometry/point.h>

namespace hrbf_surf {

struct CubicRBF {
  static f64 phi(f64 r, f64 e = 0) { return r * r * r; }
  static f64 ddr(f64 r, f64 e = 0) { return 3 * r * r; }
  static f64 d2dr2(f64 r, f64 e = 0) { return 6 * r; }
  static f64 ddx(f64 dx, f64 r, f64 e = 0) { return dx * 3 * r; }
  static f64 d2dx2(f64 dx, f64 r, f64 e = 0) {
    if (hermes::numbers::cmp::is_zero(r))
      return 0;
    return 3 * (r + dx * dx / r);
  }
  static f64 d2dxy(f64 dx, f64 dy, f64 r, f64 e = 0) {
    if (hermes::numbers::cmp::is_zero(r))
      return 0;
    return 3 * dx * dy / r;
  }
};

class HRBFSystem {
public:
  static Result<HRBFSystem>
  from(const std::vector<hermes::geo::point3d> &positions,
       const std::vector<hermes::geo::normal3d> &normals,
       const std::vector<h_index> &ids);

  static Result<HRBFSystem>
  from(const std::vector<hermes::geo::point3d> &positions,
       const std::vector<hermes::geo::normal3d> &normals);

  // eval
  f64 operator()(const hermes::geo::point3d &p) const;

private:
  HRBF_fit<f64, 3, Rbf_pow3<f64>> solver_;
  // Eigen::Matrix<f64, 3, Eigen::Dynamic> centers_;
  // Eigen::Matrix<f64, Eigen::Dynamic, 1> alphas_;
  // Eigen::Matrix<f64, 3, Eigen::Dynamic> betas_;
};

} // namespace hrbf_surf
