/* Copyright (c) 2025, FilipeCN.
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

/// \file   naiades_py.cpp
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2025-06-07
/// \brief  Python interface for Naiades lib.

#include <hermes/base/str.h>
#include <naiades/sampling/sampler.h>
#include <naiades/solvers/convection.h>
#include <naiades/solvers/smoke_solver.h>
#include <naiades/utils/fields.h>
#include <naiades/utils/math.h>

#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/native_enum.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <igl/cotmatrix.h>

namespace py = pybind11;

using py_Element = py::native_enum<naiades::core::Element::Type>;
using py_array_f32 = py::array_t<f32, py::array::c_style>;
using py_array_u32 = py::array_t<u32, py::array::c_style>;

/*
void printCallback(const hermes::cstr &m) {
  py::print(m.str());
  std::cout << std::flush;
}

void initHermes(bool verbose = false) {
  std::cout << std::unitbuf;
  hermes::io::Logger::addOptions(hermes::io ::logger_option_bits::abbreviate);
  hermes::io::Logger::setLogCallback(hermes::io::Logger::Level::info,
                                     printCallback);
  if (verbose) {
    hermes::io::Logger::setLogCallback([](const hermes::cstr &m,
                                          hermes::io::Logger::Level level,
                                          hermes::io::logger_options o) {
      HERMES_UNUSED_VARIABLE(level);
      HERMES_UNUSED_VARIABLE(o);
      py::print(m.str());
      std::cout << std::flush;
    });
  } else {
    hermes::io::Logger::setLogCallback([](const hermes::cstr &m,
                                          hermes::io::Logger::Level level,
                                          hermes::io::logger_options o) {
      HERMES_UNUSED_VARIABLE(m);
      HERMES_UNUSED_VARIABLE(level);
      HERMES_UNUSED_VARIABLE(o);
    });
  }
}*/

template <typename T>
py::array_t<T, py::array::c_style>
scalar_pyarray(const naiades::core::FieldRef<T> &data) {
  std::vector<h_size> shape = {data.size()};
  std::vector<h_size> strides = {sizeof(T)};
  return py::array_t<T, py::array::c_style>(shape, strides, data.data());
}

template <typename DataType>
py_array_f32 vector_pyarray(const std::vector<DataType> &data) {
  std::vector<f32> py_data(data.size() * 3);
  std::vector<h_size> shape = {data.size(), 3};
  std::vector<h_size> strides = {3 * sizeof(f32), sizeof(f32)};
  for (h_size i = 0; i < data.size(); ++i) {
    py_data[i * 3 + 0] = data[i].x;
    py_data[i * 3 + 1] = data[i].y;
    py_data[i * 3 + 2] = 0;
  }
  return py_array_f32(shape, strides, py_data.data());
}

struct py_Mesh {

  static py_Mesh from(const naiades::geo::Grid2 &grid,
                      naiades::core::Element element) {
    py_Mesh mesh;

    mesh.face_centers =
        vector_pyarray(grid.positions(naiades::core::Element::FACE));
    mesh.cell_centers =
        vector_pyarray(grid.positions(naiades::core::Element::CELL));
    mesh.vertex_centers =
        vector_pyarray(grid.positions(naiades::core::Element::VERTEX));

    auto g_indices = grid.indices(element, naiades::core::Element::VERTEX);
    std::vector<u32> data(g_indices.size() * 4);
    std::vector<h_size> shape = {
        grid.elementCount(naiades::core::Element::CELL), 4};
    std::vector<h_size> strides = {4 * sizeof(u32), sizeof(u32)};
    for (h_size i = 0; i < g_indices.size(); ++i)
      for (h_size j = 0; j < 4; ++j)
        data[i * 4 + j] = g_indices[i][j];
    mesh.cells = py_array_u32(shape, strides, data.data());

    return mesh;
  }

  py_array_f32 face_centers;
  py_array_f32 cell_centers;
  py_array_f32 vertex_centers;
  py_array_u32 cells;
};

struct StableFluids2_py {
  StableFluids2_py(bool verbose = false) {
    grid_.setSize({50, 50});
    grid_.setCellSize(1.f / 50);

    solver_ =
        naiades::solvers::SmokeSolver2::Config().setGrid(grid_).build().value();

    auto u = solver_.u();
    auto v = solver_.v();
    auto d = solver_.density();

    naiades::utils::setField<f32>(
        grid_, u, [&](const hermes::geo::point2 &p) -> f32 { return 0; });

    naiades::utils::setField<f32>(
        grid_, v, [&](const hermes::geo::point2 &p) -> f32 { return 0; });

    naiades::utils::setField<f32>(
        grid_, d, [&](const hermes::geo::point2 &p) -> f32 {
          return naiades::utils::sdf::sphere(hermes::geo::point2(0.5, 0.75),
                                             0.15, p) <= 0;
        });

    HERMES_UNUSED_VARIABLE(verbose);
    fields_.add<hermes::geo::vec2>(naiades::core::Element::CELL,
                                   {"cell_velocity"});
    fields_.add<f32>(naiades::core::Element::CELL,
                     {"density_0", "density_1", "cell_R", "cell_G", "cell_B"});
    fields_.add<f32>(naiades::core::Element::VERTEX, {"gaussian"});
    fields_.add<f32>(naiades::core::Element::X_FACE, {"v"});
    fields_.add<f32>(naiades::core::Element::Y_FACE, {"u"});

    fields_.setElementCountFrom(&grid_);

    auto cell_velocity = fields_.get<f32>("density_0").value();
    naiades::utils::setField<f32>(
        grid_, cell_velocity, [&](const hermes::geo::point2 &p) -> f32 {
          return naiades::utils::sdf::sphere(hermes::geo::point2(0.5, 0.75),
                                             0.15, p) <= 0;
        });

    cell_velocity = fields_.get<f32>("density_1").value();
    naiades::utils::setField<f32>(
        grid_, cell_velocity, [&](const hermes::geo::point2 &p) -> f32 {
          return naiades::utils::sdf::sphere(hermes::geo::point2(0.5, 0.75),
                                             0.15, p) <= 0;
        });

    auto uf = fields_.get<f32>("u").value();
    naiades::utils::setField<f32>(
        grid_, uf, [&](const hermes::geo::point2 &p) -> f32 {
          return naiades::utils::zalesak(p, {0.5f, 0.5f},
                                         hermes::math::constants::two_pi)
              .x;
        });

    auto vf = *fields_.get<f32>("v");
    naiades::utils::setField<f32>(
        grid_, vf, [&](const hermes::geo::point2 &p) -> f32 {
          return naiades::utils::zalesak(p, {0.5f, 0.5f},
                                         hermes::math::constants::two_pi)
              .y;
        });
  }

  void step(f32 timestep) {
    solver_.step(timestep);

    auto u = *fields_.get<f32>("u");
    auto v = *fields_.get<f32>("v");
    std::string names[] = {"density_0", "density_1"};
    auto src_d = *fields_.get<f32>(names[(frame_ + 0) % 2]);
    auto dst_d = *fields_.get<f32>(names[(frame_ + 1) % 2]);

    auto f = [](const naiades::geo::Grid2 &grid,
                const naiades::core::FieldCRef<f32> &field,
                const hermes::geo::point2 &p) -> f32 {
      auto stencil =
          naiades::sampling::Stencil::bilinear(grid, field.element(), p);
      return stencil.evaluate(field);
    };

    naiades::solvers::advect<f32>(grid_, u, v, f, timestep, src_d, dst_d);

    auto cell_velocity = *fields_.get<hermes::geo::vec2>("cell_velocity");
    naiades::utils::zalesakVelocityField(grid_, cell_velocity, {0.5f, 0.5f},
                                         hermes::math::constants::two_pi);

    auto cell_r = *fields_.get<f32>("cell_R");
    naiades::utils::setField<f32>(
        grid_, cell_r, [&](const hermes::geo::point2 &p) -> f32 {
          return naiades::utils::zalesak(p, {0.5f, 0.5f},
                                         hermes::math::constants::two_pi)
              .x;
        });

    frame_++;
  }

  py_array_f32 getDensity() {
    auto density = fields_.get<f32>(frame_ % 2 ? "density_1" : "density_0");
    return scalar_pyarray(density.value());
  }

  py_array_f32 getStaggeredVelocityField() {
    // auto *u = fields_.get<f32>("u");
    // auto *v = fields_.get<f32>("v");
    auto u = solver_.u();
    auto v = solver_.v();

    std::vector<hermes::geo::vec2> data(
        grid_.elementCount(naiades::core::Element::Type::FACE));

    for (h_size i = 0; i < v.size(); ++i) {
      auto flat_index =
          grid_.flatIndexOffset(naiades::core::Element::Type::X_FACE) + i;
      data[flat_index].x = 0;
      data[flat_index].y = v[i];
    }
    for (h_size i = 0; i < u.size(); ++i) {
      auto flat_index =
          grid_.flatIndexOffset(naiades::core::Element::Type::Y_FACE) + i;
      data[flat_index].x = u[i];
      data[flat_index].y = 0;
    }
    return vector_pyarray(data);
  }

  py_array_f32 getScalarField(const std::string &name) {
    auto field = fields_.get<f32>(name);
    if (!field)
      throw std::runtime_error("Field not found!");
    return scalar_pyarray(*field);
  }

  py::array_t<float> sampleFloatField(const std::string &name,
                                      py::array_t<f32> &arr) {
    auto field = fields_.get<f32>(name);
    if (!field)
      throw std::runtime_error("Field not found!");

    // access array data and dimensions
    auto buf = arr.request();
    const float *ptr = static_cast<const float *>(buf.ptr);
    if (buf.ndim != 1)
      throw std::runtime_error("Only 1D arrays are supported.");

    h_size positions_count = buf.shape[0] / 2;

    std::vector<hermes::geo::point2> samples;
    for (h_size i = 0; i < positions_count; ++i)
      samples.emplace_back(ptr[i * 2], ptr[i * 2 + 1]);

    // sample field
    auto samples_field = naiades::sampling::sample(grid_, *field, samples);

    std::vector<f32> data(positions_count);
    std::vector<h_size> shape = {positions_count};
    std::vector<h_size> strides = {sizeof(f32)};
    for (h_size i = 0; i < positions_count; ++i)
      data[i] = samples_field[i];

    return py::array_t<f32>(shape,        // Shape (e.g., {3, 4})
                            strides,      // Strides (e.g., {32, 8} bytes)
                            data.data()); // Pointer to data
  }

  py::array_t<float> sampleVectorField(const std::string &name,
                                       py::array_t<f32> &arr) {
    auto field = fields_.get<hermes::geo::vec2>(name);
    if (!field)
      throw std::runtime_error("Field not found!");

    // access array data and dimensions
    auto buf = arr.request();
    const float *ptr = static_cast<const float *>(buf.ptr);
    if (buf.ndim != 1)
      throw std::runtime_error("Only 1D arrays are supported.");

    h_size positions_count = buf.shape[0] / 2;

    std::vector<hermes::geo::point2> samples;
    for (h_size i = 0; i < positions_count; ++i)
      samples.emplace_back(ptr[i * 2], ptr[i * 2 + 1]);

    // sample field
    auto samples_field = naiades::sampling::sample(grid_, *field, samples);

    std::vector<f32> data(positions_count * 2);
    std::vector<h_size> shape = {positions_count, 2};
    std::vector<h_size> strides = {2 * sizeof(f32), sizeof(f32)};
    for (h_size i = 0; i < positions_count; ++i) {
      data[i * 2 + 0] = samples_field[i].x;
      data[i * 2 + 1] = samples_field[i].y;
    }

    return py::array_t<f32>(shape,        // Shape (e.g., {3, 4})
                            strides,      // Strides (e.g., {32, 8} bytes)
                            data.data()); // Pointer to data
  }

  py::array_t<f32> getPositions(naiades::core::Element loc) {
    auto positions_count = grid_.elementCount(loc);
    std::vector<f32> data(positions_count * 2);

    for (auto ij : hermes::range2(grid_.resolution(loc))) {
      auto flat_ij = grid_.flatIndex(loc, ij);
      auto wp = grid_.position(loc, ij);
      data[flat_ij * 2 + 0] = wp.x;
      data[flat_ij * 2 + 1] = wp.y;
    }

    std::vector<h_size> shape = {positions_count, 2};
    std::vector<h_size> strides = {2 * sizeof(f32), sizeof(f32)};
    return py::array_t<f32>(shape, strides, data.data());
  }

  py_Mesh mesh(naiades::core::Element::Type element_type) {
    auto m = py_Mesh::from(grid_, element_type);
    return m;
  }

  naiades::geo::Grid2 grid_;
  naiades::core::FieldSet fields_;
  naiades::solvers::SmokeSolver2 solver_;
  u32 frame_{0};
};

#define PY_ENUM_VALUE(T, V) .value(#V, T::V)

PYBIND11_MODULE(naiades_py, m) {
  m.doc() = "Naiades module";

  py_Element(m, "Element", "enum.IntEnum")
      PY_ENUM_VALUE(naiades::core::Element::Type, CELL)   //
      PY_ENUM_VALUE(naiades::core::Element::Type, U_FACE) //
      PY_ENUM_VALUE(naiades::core::Element::Type, V_FACE) //
          .finalize();

  py::class_<py_Mesh>(m, "Mesh")
      .def_readonly("cell_centers", &py_Mesh::cell_centers)
      .def_readonly("face_centers", &py_Mesh::face_centers)
      .def_readonly("vertex_centers", &py_Mesh::vertex_centers)
      .def_readonly("cells", &py_Mesh::cells);

  py::class_<StableFluids2_py>(m, "StableFluids2")
      .def(py::init([](bool verbose) { return new StableFluids2_py(verbose); }))
      .def("get_scalar_field", &StableFluids2_py::getScalarField, "")
      .def("get_density_field", &StableFluids2_py::getDensity, "")
      .def("get_stag_velocity_field",
           &StableFluids2_py::getStaggeredVelocityField, "")
      .def("step", &StableFluids2_py::step, "")
      .def("sample_float_field", &StableFluids2_py::sampleFloatField, "")
      .def("sample_vector_field", &StableFluids2_py::sampleVectorField, "")
      .def("get_positions", &StableFluids2_py::getPositions, "")
      .def("get_mesh", &StableFluids2_py::mesh, "");
}
