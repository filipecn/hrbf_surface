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

using namespace hrbf_surf;

using py_array_Scalar = py::array_t<Scalar, py::array::c_style>;
using py_array_u32 = py::array_t<u64, py::array::c_style>;

template <typename DataType>
py_array_Scalar vector_pyarray(const std::vector<DataType> &data) {
  std::vector<Scalar> py_data(data.size() * 3);
  std::vector<h_size> shape = {data.size(), 3};
  std::vector<h_size> strides = {3 * sizeof(Scalar), sizeof(Scalar)};
  for (h_size i = 0; i < data.size(); ++i) {
    py_data[i * 3 + 0] = data[i].x;
    py_data[i * 3 + 1] = data[i].y;
    py_data[i * 3 + 2] = data[i].z;
  }
  return py_array_Scalar(shape, strides, py_data.data());
}

template <typename T> std::vector<T> from_pyarray(const py_array_Scalar &arr) {
  auto buf = arr.request();
  const Scalar *ptr = static_cast<const Scalar *>(buf.ptr);
  if (buf.ndim != 2)
    throw std::runtime_error("Only 2D arrays are supported.");
  h_size positions_count = buf.shape[0];
  std::vector<T> values;
  for (h_size i = 0; i < positions_count; ++i)
    values.emplace_back(ptr[i * 3], ptr[i * 3 + 1], ptr[i * 3 + 2]);
  return values;
}

struct py_PointCloud {
  py_array_Scalar positions;
  py_array_Scalar normals;
};

py_PointCloud loadPointCloud(const std::string &filepath) {

  auto pcl = hrbf_surf::io::loadPCLFromPly(filepath);

  py_PointCloud py_pcl;
  py_pcl.positions = vector_pyarray<Point>((*pcl)->positions());
  py_pcl.normals = vector_pyarray<Vector>((*pcl)->normals());

  return py_pcl;
}

py_PointCloud sphere(Scalar radius, h_index res) {
  Scalar step_size = hermes::math::constants::pi / res;
  std::vector<Point> positions;
  std::vector<Vector> normals;
  for (Scalar phi = step_size; phi < hermes::math::constants::pi - step_size;
       phi += step_size)
    for (Scalar theta = 0.0;
         theta < hermes::math::constants::two_pi - step_size;
         theta += step_size) {
      positions.emplace_back(radius * std::sin(phi) * std::cos(theta),
                             radius * std::sin(phi) * std::sin(theta),
                             radius * std::cos(phi));
      Vector n(std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta),
               std::cos(phi));
      n.normalize();
      normals.emplace_back(n.x, n.y, n.z);
    }

  positions.emplace_back(Point(0.0, 0.0, radius));
  positions.emplace_back(Point(0.0, 0.0, -radius));

  normals.emplace_back(Point(0.0, 0.0, 1.0));
  normals.emplace_back(Point(0.0, 0.0, -1.0));

  py_PointCloud py_pcl;
  py_pcl.positions = vector_pyarray<Point>(positions);
  py_pcl.normals = vector_pyarray<Vector>(normals);

  return py_pcl;
}

py_PointCloud square(Scalar size, h_index res) {
  std::vector<Point> positions;
  std::vector<Vector> normals;

  auto step = size / res;
  for (auto ij : hermes::range2(hermes::size2(res))) {
    positions.emplace_back(Point() + Vector(step * ij.i, step * ij.j, 0.0));
    normals.emplace_back(Vector(0.0, 0.0, 1.0));
  }

  py_PointCloud py_pcl;
  py_pcl.positions = vector_pyarray<Point>(positions);
  py_pcl.normals = vector_pyarray<Vector>(normals);

  return py_pcl;
}

PoUSurface::Ptr reconstruct(py_PointCloud py_pcl, Scalar partition_size) {
  auto positions = from_pyarray<Point>(py_pcl.positions);
  auto normals = from_pyarray<Vector>(py_pcl.normals);

  auto pcl = hrbf_surf::PointCloud::from(positions, normals);
  auto ps = hrbf_surf::PoUSurface::from(pcl, partition_size, 0.2);

  return *ps;
}

PoUSurface::Ptr reconstruct_single(py_PointCloud py_pcl) {
  auto positions = from_pyarray<Point>(py_pcl.positions);
  auto normals = from_pyarray<Vector>(py_pcl.normals);

  auto pcl = hrbf_surf::PointCloud::from(positions, normals);
  auto ps = hrbf_surf::PoUSurface::from(pcl);

  return *ps;
}

struct py_SurfaceMesh {
  Eigen::MatrixXd V; // Mesh Vertices
  Eigen::MatrixXi F; // Mesh Faces (triangles)
  Eigen::MatrixXd P; // partition vertices
  Eigen::MatrixXd B; // partition bounds
};

py_SurfaceMesh create_partition_mesh(PoUSurface::Ptr ps, Scalar voxel_size,
                                     h_index i = 0) {
  auto s = ps->partitionMesh(0, voxel_size);
  py_SurfaceMesh sm;
  sm.F = s.F;
  sm.V = s.V;
  return sm;
}

py_SurfaceMesh create_mesh(PoUSurface::Ptr ps, Scalar voxel_size) {
  auto s = ps->mesh(voxel_size);
  py_SurfaceMesh sm;
  sm.F = s.F;
  sm.V = s.V;
  return sm;
}

py_SurfaceMesh reconstruct_surface(py_PointCloud py_pcl, Scalar partition_size,
                                   Scalar overlap, Scalar voxel_size,
                                   bool single, i32 partition) {
  auto positions = from_pyarray<Point>(py_pcl.positions);
  auto normals = from_pyarray<Vector>(py_pcl.normals);

  auto pcl = hrbf_surf::PointCloud::from(positions, normals);
  hrbf_surf::PoUSurface::Ptr ps;

  if (single)
    ps = *hrbf_surf::PoUSurface::from(pcl);
  else
    ps = *hrbf_surf::PoUSurface::from(pcl, partition_size, overlap);

  py_SurfaceMesh sm;

  if (partition >= 0) {
    auto s = ps->partitionMesh(partition, voxel_size);
    sm.F = s.F;
    sm.V = s.V;
    auto part = ps->partition(partition);
    sm.P = Eigen::MatrixXd(part.indices.size(), 3);
    for (h_index i = 0; i < part.indices.size(); ++i) {
      auto p = pcl->positions()[part.indices[i]];
      sm.P(i, 0) = p.x;
      sm.P(i, 1) = p.y;
      sm.P(i, 2) = p.z;
    }
    sm.B = Eigen::MatrixXd(8, 3);
    for (h_index i = 0; i < 8; ++i) {
      auto p = part.bounds.corner(i);
      sm.B(i, 0) = p.x;
      sm.B(i, 1) = p.y;
      sm.B(i, 2) = p.z;
    }
  } else {
    auto s = ps->mesh(voxel_size);
    sm.F = s.F;
    sm.V = s.V;
  }
  return sm;
}

PYBIND11_DECLARE_HOLDER_TYPE(T, hermes::Ref<T>)

PYBIND11_MODULE(hrbf_surface_py, m) {
  m.doc() = "Naiades module";

  py::class_<py_PointCloud>(m, "PointCloud")
      .def_readonly("positions", &py_PointCloud::positions)
      .def_readonly("normals", &py_PointCloud::normals);
  py::class_<py_SurfaceMesh>(m, "SurfaceMesh")
      .def_readonly("pv", &py_SurfaceMesh::P)
      .def_readonly("bounds", &py_SurfaceMesh::B)
      .def_readonly("positions", &py_SurfaceMesh::V)
      .def_readonly("faces", &py_SurfaceMesh::F);

  m.def("load_point_cloud", &loadPointCloud);
  m.def("sphere", &sphere);
  m.def("square", &square);
  m.def("reconstruct_surface", &reconstruct_surface);
  m.def("reconstruct", &reconstruct, py::return_value_policy::reference);
  m.def("reconstruct_single", &reconstruct_single,
        py::return_value_policy::reference);
  m.def("partition_mesh", &create_partition_mesh);
  m.def("mesh", &create_mesh);
}