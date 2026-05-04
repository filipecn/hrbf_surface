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

Result<HRBFSystem>
HRBFSystem::from(const std::vector<hermes::geo::point3d> &positions,
                 const std::vector<hermes::geo::normal3d> &normals,
                 const std::vector<h_index> &ids) {

  std::vector<Eigen::Matrix<f64, 3, 1>> points(ids.size());
  std::vector<Eigen::Matrix<f64, 3, 1>> _normals(ids.size());
  for (h_index i = 0; i < ids.size(); ++i) {
    for (h_index j = 0; j < 3; ++j) {
      points[i][j] = positions[ids[i]][j];
    }
    _normals[i][0] = normals[ids[i]].x;
    _normals[i][1] = normals[ids[i]].y;
    _normals[i][2] = normals[ids[i]].z;
  }

  HRBFSystem hrbf_s;

  hrbf_s.solver_.hermite_fit(points, _normals);

  // h_size N = ids.size();
  // h_index hrbf_constraints_n = 4 * N;
  // h_index constraints_n = hrbf_constraints_n;
  // h_index coeffs_n = 4 * N;

  // hrbf_s.centers_.resize(3, N);
  // hrbf_s.betas_.resize(3, N);
  // hrbf_s.alphas_.resize(N);

  // Eigen::Matrix<f64, Eigen::Dynamic, Eigen::Dynamic> D(constraints_n,
  // coeffs_n); Eigen::Matrix<f64, Eigen::Dynamic, 1> f(constraints_n);
  // Eigen::Matrix<f64, Eigen::Dynamic, 1> x(coeffs_n);

  // for (h_index i = 0; i < N; ++i) {
  //   hrbf_s.centers_.col(i)[0] = positions[ids[i]].x;
  //   hrbf_s.centers_.col(i)[1] = positions[ids[i]].y;
  //   hrbf_s.centers_.col(i)[2] = positions[ids[i]].z;
  // }

  // for (h_index i = 0; i < N; ++i) {
  //   Eigen::Matrix<f64, 3, 1> p(positions[ids[i]].x, positions[ids[i]].y,
  //                              positions[ids[i]].z);
  //   Eigen::Matrix<f64, 3, 1> n(normals[ids[i]].x, normals[ids[i]].y,
  //                              normals[ids[i]].z);

  //   h_index io = 4 * i;
  //   f(io) = 0;
  //   f.template segment<3>(io + 1) = n;

  //   for (h_index j = 0; j < N; ++j) {
  //     h_index jo = 4 * j;
  //     auto diff = p - hrbf_s.centers_.col(j);
  //     f64 l = diff.norm();
  //     if (hermes::numbers::cmp::is_zero(l)) {
  //       D.template block<4, 4>(io, jo).setZero();
  //     } else {
  //       f64 w = CubicRBF::phi(l);
  //       f64 dw_l = CubicRBF::ddr(l) / l;
  //       f64 ddw = CubicRBF::d2dr2(l);
  //       auto g = diff * dw_l;
  //       D(io, jo) = w;
  //       D.row(io).template segment<3>(jo + 1) = g.transpose();
  //       D.col(jo).template segment<3>(io + 1) = g;
  //       D.template block<3, 3>(io + 1, jo + 1) =
  //           (ddw - dw_l) / (l * l) * (diff * diff.transpose());
  //       D.template block<3, 3>(io + 1, jo + 1).diagonal().array() += dw_l;
  //     }
  //   }
  // }

  // HERMES_LOG_VARIABLE(D.hasNaN());
  // HERMES_LOG_VARIABLE(f.hasNaN());

  // x = D.lu().solve(f);
  // HERMES_LOG_VARIABLE(x.hasNaN());
  // Eigen::Map<Eigen::Matrix<f64, 4, Eigen::Dynamic>> mx(x.data(), 4, N);

  // hrbf_s.alphas_ = mx.row(0);
  // hrbf_s.betas_ = mx.template bottomRows<3>();

  return Result<HRBFSystem>(std::move(hrbf_s));
}

Result<HRBFSystem>
HRBFSystem::from(const std::vector<hermes::geo::point3d> &positions,
                 const std::vector<hermes::geo::normal3d> &normals) {
  std::vector<h_index> ids;
  for (h_index i = 0; i < positions.size(); ++i)
    ids.emplace_back(i);
  return HRBFSystem::from(positions, normals, ids);
}

f64 HRBFSystem::operator()(const hermes::geo::point3d &p) const {
  Eigen::Matrix<f64, 3, 1> v;
  v[0] = p.x;
  v[1] = p.y;
  v[2] = p.z;

  return solver_.eval(v);
  // f64 ret = 0;
  // h_index N = centers_.cols();

  // for (h_index i = 0; i < N; ++i) {
  //   Eigen::Matrix<f64, 3, 1> x;

  //   x[0] = p.x;
  //   x[1] = p.y;
  //   x[2] = p.z;

  //   auto diff = x - centers_.col(i);
  //   auto l = diff.norm();

  //   if (l > 0) {
  //     ret += alphas_(i) * CubicRBF::phi(l);
  //     ret += betas_.col(i).dot(diff) * CubicRBF::ddr(l) / l;
  //   }
  // }
  // return ret;
}

} // namespace hrbf_surf
