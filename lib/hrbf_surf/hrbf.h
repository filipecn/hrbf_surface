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
#include <hrbf_surf/types.h>

#include <Eigen/Dense>
#include <vector>

namespace hrbf_surf {

struct CubicRBF {
  static Scalar phi(Scalar r, Scalar e = 0) { return r * r * r; }
  static Scalar ddr(Scalar r, Scalar e = 0) { return 3 * r * r; }
  static Scalar d2dr2(Scalar r, Scalar e = 0) { return 6 * r; }
  static Scalar ddx(Scalar dx, Scalar r, Scalar e = 0) { return dx * 3 * r; }
  static Scalar d2dx2(Scalar dx, Scalar r, Scalar e = 0) {
    if (hermes::numbers::cmp::is_zero(r))
      return 0;
    return 3 * (r + dx * dx / r);
  }
  static Scalar d2dxy(Scalar dx, Scalar dy, Scalar r, Scalar e = 0) {
    if (hermes::numbers::cmp::is_zero(r))
      return 0;
    return 3 * dx * dy / r;
  }
};

class HRBFSystem2 {
public:
  static Result<HRBFSystem2> from(const std::vector<Point> &positions,
                                  const std::vector<Vector> &normals,
                                  const std::vector<h_index> &ids);

  static Result<HRBFSystem2> from(const std::vector<Point> &positions,
                                  const std::vector<Vector> &normals);

  // eval
  Scalar operator()(const Point &p) const;

private:
  HRBF_fit<Scalar, 3, Rbf_pow3<Scalar>> solver_;
};

template <typename RBFKernel> class HRBFSystem {
public:
  HRBFSystem() = default;

  void init(const std::vector<Point> &centers,
            const std::vector<Vector> &normals, const std::vector<size_t> &ids,
            Scalar e = 0) {
#define P(I) centers[ids[I]]
#define D(I, J) hermes::geo::distance(P(I), P(J))

#define UPPER_diag BLOCK_I_OFFSET + i, BLOCK_J_OFFSET + i
#define UPPER_upper BLOCK_I_OFFSET + i, BLOCK_J_OFFSET + j
#define UPPER_lower BLOCK_I_OFFSET + j, BLOCK_J_OFFSET + i
#define LOWER_diag BLOCK_J_OFFSET + i, BLOCK_I_OFFSET + i
#define LOWER_upper BLOCK_J_OFFSET + i, BLOCK_I_OFFSET + j
#define LOWER_lower BLOCK_J_OFFSET + j, BLOCK_I_OFFSET + i
#define DIAG_diag BLOCK_I_OFFSET + i, BLOCK_I_OFFSET + i
#define DIAG_upper BLOCK_I_OFFSET + i, BLOCK_I_OFFSET + j
#define DIAG_lower BLOCK_I_OFFSET + j, BLOCK_I_OFFSET + i

    //                        p  px py pz
    //     N    N    N    N   1  1  1  1
    // N  PHI -DDX -DDY -DDZ  1  X  Y  Z      w     alpha
    // N  DDX -DXX -DXY -DXZ  0  1  0  0      wx    alpha_x
    // N  DDY -DXY -DYY -DYZ  0  0  1  0   x  wy =  alpha_y
    // N  DDZ -DXZ -DYZ -DZZ  0  0  0  1      wz    alpha_z
    // 1   1    0    0    0   0  0  0  0      g     0
    // 1   X    1    0    0   0  0  0  0      gx    0
    // 1   Y    0    1    0   0  0  0  0      gy    0
    // 1   Z    0    0    1   0  0  0  0      gz    0
    // setup matrix size
    size_t n = ids.size();
    size_t N = n * 4 + POLY;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, N);

    // p
    if (POLY > 0) {
      for (size_t i = 0; i < n; ++i) {
        // 1
        int offset = n * 4;
        for (size_t dim = 0; dim <= 3; ++dim) {
          A(i + n * dim, offset + dim) = A(offset + dim, i + n * dim) = 1;
          // X  Y  Z
          if (dim > 0) {
            A(i, offset + dim) = A(offset + dim, i) = P(i)[dim - 1];
          }
        }
      }
    }

    // PHI
    for (size_t i = 0; i < n; ++i) {
      A(i, i) = RBFKernel::phi(0, e);
      for (size_t j = i + 1; j < n; ++j) {
        A(i, j) = A(j, i) = RBFKernel::phi(D(i, j), e);
      }
    }
    // DD?
    for (int dim = 0; dim < 3; ++dim) {
      int BLOCK_I_OFFSET = 0;
      int BLOCK_J_OFFSET = dim * n + n;
      for (int i = 0; i < n; ++i) {
        // DD? diag
        A(UPPER_diag) = -RBFKernel::ddx(0, 0, e);
        A(LOWER_diag) = -A(UPPER_diag);
        for (int j = i + 1; j < n; ++j) {
          auto d = P(j) - P(i);

          // d/d dim (phi(i,j)) -> -DD?(I, J)
          A(UPPER_upper) = RBFKernel::ddx(d[dim], d.length(), e);
          A(UPPER_lower) = -A(UPPER_upper);

          // d/d dim (phi(i,j)) -> DD?(J, I) = -DD?(I, J)
          A(LOWER_upper) = A(UPPER_lower);
          A(LOWER_lower) = A(UPPER_upper);
        }
      }
    }

    // DXX
    for (int dim = 0; dim < 3; ++dim) {
      int BLOCK_I_OFFSET = dim * n + n;
      for (int i = 0; i < n; ++i) {
        A(DIAG_diag) = -RBFKernel::d2dx2(0, 0, e);
        for (int j = i + 1; j < n; ++j) {
          // d/d dim (phi(i,j)) -> -DD?(I, J)
          auto d = P(j) - P(i);
          A(DIAG_upper) = A(DIAG_lower) =
              -RBFKernel::d2dx2(d[dim], d.length(), e);
        }
      }
    }

    // DXY
    for (int dim_i = 0; dim_i < 3; ++dim_i) {
      int BLOCK_I_OFFSET = dim_i * n + n;
      for (int dim_j = dim_i + 1; dim_j < 3; ++dim_j) {
        int BLOCK_J_OFFSET = dim_j * n + n;
        for (int i = 0; i < n; ++i) {
          for (int j = 0; j < n; ++j) {
            auto d = P(j) - P(i);

            A(UPPER_upper) = A(UPPER_lower) = A(LOWER_upper) = A(LOWER_lower) =
                -RBFKernel::d2dxy(d[dim_i], d[dim_j], d.length(), e);
          }
        }
      }
    }

    A = A.inverse();
#undef D
#undef P

    w.resize(N);

    Eigen::VectorXd b(N);

#define G(D, I) normals[ids[I]][D]

    for (size_t i = 0; i < n; ++i)
      b[i] = 0.0; // F(i);
    for (size_t dim = 0; dim < 3; ++dim)
      for (size_t i = 0; i < n; ++i)
        b[n + dim * n + i] = G(dim, i);

    for (size_t p = 0; p < POLY; ++p)
      b[4 * n + p] = 0;

    w = A * b;

#undef G
#undef F
  }

  Scalar operator()(const std::vector<Point> &centers,
                    const std::vector<size_t> &ids, const Point &p,
                    Scalar e = 0) const {
#define C(I) centers[ids[I]]

    size_t n = ids.size();
    size_t N = n * 4 + POLY;

    if (w.rows() != N) {
      HERMES_ERROR("bad system size {} != {}", N, w.rows());
      return {};
    }

    Eigen::VectorXd v;
    auto wt = w.transpose();
    v.resize(N);

    for (size_t i = 0; i < n; ++i)
      v[i] = RBFKernel::phi(hermes::geo::distance(p, C(i)), e);
    for (size_t dim = 0; dim < 3; ++dim) {
      for (size_t i = 0; i < n; ++i) {
        auto d = p - C(i);
        v[n + dim * n + i] = -RBFKernel::ddx(d[dim], d.length(), e);
      }
      if (POLY == 4) {
        v[4 * n + 0] = 1;
        v[4 * n + dim + 1] = p[dim];
      }
    }

    return wt * v;
#undef C
  }

  bool empty() const { return w.size() == 0; }
  bool hasNaN() const { return w.hasNaN(); }

private:
  Eigen::VectorXd w;

  h_index POLY = 4;
};

template <typename RBFKernel> class RBFSystem {
public:
  void init(const std::vector<Point> &centers, const std::vector<size_t> &ids,
            Scalar e = 0) {
#define P(I) centers[ids[I]]
#define D(I, J) hermes::geo::distance(P(I), P(J))

    //     N
    // N  PHI    w   alpha

    // setup matrix size
    size_t n = ids.size();
    size_t N = n;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, N);

    // PHI
    for (size_t i = 0; i < n; ++i) {
      A(i, i) = RBFKernel::phi(0, e);
      for (size_t j = i + 1; j < n; ++j) {
        A(i, j) = A(j, i) = RBFKernel::phi(D(i, j), e);
      }
    }

    A = A.inverse();

    w.resize(N);

    Eigen::VectorXd b(N);
#undef D
#undef P

#define G(D, I) grad_values[ids[I]][D]

    if (w.rows() != N || b.rows() != N) {
      HERMES_ERROR("bad system size {} != {} != {}", N, w.rows(), b.rows());
      return;
    }

    for (size_t i = 0; i < n; ++i)
      b[i] = 0.0; // F(i);

    w = A * b;

#undef G
#undef F
  }

  Scalar operator()(const std::vector<Point> &centers,
                    const std::vector<size_t> &ids, const Point &p,
                    Scalar e = 0) const {

    size_t n = ids.size();
    Eigen::VectorXd v;
    auto wt = w.transpose();
    v.resize(n);

    for (size_t i = 0; i < n; ++i)
      v[i] = RBFKernel::phi(hermes::geo::distance(p, centers[ids[i]]), e);

    return wt * v;
  }

  std::vector<Scalar> eval(const std::vector<Point> &centers,
                           const std::vector<size_t> &ids,
                           const std::vector<Point> &points,
                           Scalar e = 0) const {
#define C(I) centers[ids[I]]

    size_t n = ids.size();
    size_t N = n;

    if (w.rows() != N) {
      HERMES_ERROR("bad system size {} != {}", N, w.rows());
      return {};
    }

    Eigen::VectorXd v;
    auto wt = w.transpose();
    v.resize(N);

    std::vector<Scalar> r;

    bool first = true;
    for (const auto &p : points) {
      for (size_t i = 0; i < n; ++i)
        v[i] = RBFKernel::phi(hermes::geo::distance(p, C(i)), e);

      r.emplace_back(wt * v);
    }

    return r;

#undef C
  }

  bool empty() const { return w.size() == 0; }
  bool hasNaN() const { return w.hasNaN(); }

private:
  Eigen::VectorXd w;
};

} // namespace hrbf_surf
