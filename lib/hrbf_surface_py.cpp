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

/// \file   hrbf_surface_py.cpp
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-05-03

#include <hrbf_surf/io.h>
#include <hrbf_surf/point_cloud.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

using py_array_f64 = py::array_t<f64, py::array::c_style>;
using py_array_u32 = py::array_t<u64, py::array::c_style>;

template <typename DataType>
py_array_f64 vector_pyarray(const std::vector<DataType> &data) {
  std::vector<f64> py_data(data.size() * 3);
  std::vector<h_size> shape = {data.size(), 3};
  std::vector<h_size> strides = {3 * sizeof(f64), sizeof(f64)};
  for (h_size i = 0; i < data.size(); ++i) {
    py_data[i * 3 + 0] = data[i].x;
    py_data[i * 3 + 1] = data[i].y;
    py_data[i * 3 + 2] = data[i].z;
  }
  return py_array_f64(shape, strides, py_data.data());
}

struct py_PointCloud {
  py_array_f64 positions;
  py_array_f64 normals;
};

py_PointCloud loadPointCloud(const std::string &filepath) {

  auto pcl = hrbf_surf::io::loadPCLFromPly(filepath);

  py_PointCloud py_pcl;
  py_pcl.positions = vector_pyarray<hermes::geo::point3d>((*pcl).positions());
  py_pcl.normals = vector_pyarray<hermes::geo::normal3d>((*pcl).normals());

  return py_pcl;
}

py_PointCloud sphere(f64 radius, h_index res) {
  f64 step_size = hermes::math::constants::pi / res;
  std::vector<hermes::geo::point3d> positions;
  std::vector<hermes::geo::normal3d> normals;
  for (f64 phi = 0.0; phi <= hermes::math::constants::pi; phi += step_size)
    for (f64 theta = 0.0; theta <= hermes::math::constants::two_pi;
         theta += step_size) {
      positions.emplace_back(radius * std::sin(phi) * std::cos(theta),
                             radius * std::sin(phi) * std::sin(theta),
                             radius * std::cos(phi));
      normals.emplace_back(std::sin(phi) * std::cos(theta),
                           std::sin(phi) * std::sin(theta), std::cos(phi));
    }

  py_PointCloud py_pcl;
  py_pcl.positions = vector_pyarray<hermes::geo::point3d>(positions);
  py_pcl.normals = vector_pyarray<hermes::geo::normal3d>(normals);

  return py_pcl;
}

struct py_SurfaceMesh {
  Eigen::MatrixXd V; // Mesh Vertices
  Eigen::MatrixXi F; // Mesh Faces (triangles)
};

template <typename T> std::vector<T> from_pyarray(const py_array_f64 &arr) {
  auto buf = arr.request();
  const f64 *ptr = static_cast<const f64 *>(buf.ptr);
  if (buf.ndim != 2)
    throw std::runtime_error("Only 2D arrays are supported.");
  h_size positions_count = buf.shape[0];
  std::vector<T> values;
  for (h_size i = 0; i < positions_count; ++i)
    values.emplace_back(ptr[i * 3], ptr[i * 3 + 1], ptr[i * 3 + 2]);
  return values;
}

py_SurfaceMesh reconstruct(const py_PointCloud &py_pcl, f64 voxel_size) {
  auto positions = from_pyarray<hermes::geo::point3d>(py_pcl.positions);
  auto normals = from_pyarray<hermes::geo::normal3d>(py_pcl.normals);

  auto pcl = hrbf_surf::PointCloud::from(positions, normals);
  auto ps = hrbf_surf::PoUSurface::from(pcl); //, 2.0, 0.2);
  auto s = (*ps).partitionMesh(0, voxel_size);
  py_SurfaceMesh sm;
  sm.F = s.F;
  sm.V = s.V;
  return sm;
}

PYBIND11_MODULE(hrbf_surface_py, m) {
  m.doc() = "Naiades module";

  py::class_<py_PointCloud>(m, "PointCloud")
      .def_readonly("positions", &py_PointCloud::positions)
      .def_readonly("normals", &py_PointCloud::normals);
  py::class_<py_SurfaceMesh>(m, "SurfaceMesh")
      .def_readonly("positions", &py_SurfaceMesh::V)
      .def_readonly("faces", &py_SurfaceMesh::F);

  m.def("load_point_cloud", &loadPointCloud);
  m.def("pcl_sphere", &sphere);
  m.def("reconstruct", &reconstruct);
}