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

/// \file   io.cpp
/// \author FilipeCN (filipedecn@gmail.com)
/// \date   2026-05-01

#include <hrbf_surf/io.h>

#include <fstream>
#include <igl/tinyply.h>

namespace hrbf_surf::io {

class manual_timer {
  std::chrono::high_resolution_clock::time_point t0;
  double timestamp{0.0};

public:
  void start() { t0 = std::chrono::high_resolution_clock::now(); }
  void stop() {
    timestamp = std::chrono::duration<double>(
                    std::chrono::high_resolution_clock::now() - t0)
                    .count() *
                1000.0;
  }
  const double &get() { return timestamp; }
};

struct memory_buffer : public std::streambuf {
  char *p_start{nullptr};
  char *p_end{nullptr};
  size_t size;

  memory_buffer(char const *first_elem, size_t size)
      : p_start(const_cast<char *>(first_elem)), p_end(p_start + size),
        size(size) {
    setg(p_start, p_start, p_end);
  }

  pos_type seekoff(off_type off, std::ios_base::seekdir dir,
                   std::ios_base::openmode which) override {
    if (dir == std::ios_base::cur) {
      char *new_pos = gptr() + off;
      if (new_pos < p_start)
        new_pos = p_start;
      if (new_pos > p_end)
        new_pos = p_end;
      setg(p_start, new_pos, p_end);
    } else {
      char *new_pos = (dir == std::ios_base::beg ? p_start : p_end) + off;
      if (new_pos < p_start)
        new_pos = p_start;
      if (new_pos > p_end)
        new_pos = p_end;
      setg(p_start, new_pos, p_end);
    }
    return gptr() - p_start;
  }

  pos_type seekpos(pos_type pos, std::ios_base::openmode which) override {
    return seekoff(pos, std::ios_base::beg, which);
  }
};

struct memory_stream : virtual memory_buffer, public std::istream {
  memory_stream(char const *first_elem, size_t size)
      : memory_buffer(first_elem, size),
        std::istream(static_cast<std::streambuf *>(this)) {}
};

inline std::vector<uint8_t> read_file_binary(const std::string &pathToFile) {
  std::ifstream file(pathToFile, std::ios::binary);
  std::vector<uint8_t> fileBufferBytes;

  if (file.is_open()) {
    file.seekg(0, std::ios::end);
    size_t sizeBytes = file.tellg();
    file.seekg(0, std::ios::beg);
    fileBufferBytes.resize(sizeBytes);
    if (file.read((char *)fileBufferBytes.data(), sizeBytes))
      return fileBufferBytes;
  } else
    throw std::runtime_error("could not open binary ifstream to path " +
                             pathToFile);
  return fileBufferBytes;
}

Result<PointCloud::Ptr> loadPCLFromPly(std::filesystem::path filepath) {
  std::unique_ptr<std::istream> file_stream;
  std::vector<uint8_t> byte_buffer;

  const bool preload_into_memory = true;

  std::vector<Point> verts;
  std::vector<Vector> norms;

  try {
    // For most files < 1gb, preloading the entire file upfront into memory and
    // wrapping it into a stream is a net win for parsing speed (~30-40% faster)
    if (preload_into_memory) {
      byte_buffer = read_file_binary(filepath.string());
      file_stream.reset(
          new memory_stream((char *)byte_buffer.data(), byte_buffer.size()));
    } else {
      file_stream.reset(new std::ifstream(filepath, std::ios::binary));
    }

    if (!file_stream || file_stream->fail())
      throw std::runtime_error("file_stream failed to open " +
                               filepath.string());

    file_stream->seekg(0, std::ios::end);
    const float size_mb = file_stream->tellg() * float(1e-6);
    file_stream->seekg(0, std::ios::beg);

    igl::tinyply::PlyFile file;
    file.parse_header(*file_stream);

    // std::cout << "\t[ply_header] Type: "
    //           << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
    // for (const auto &c : file.get_comments())
    //   std::cout << "\t[ply_header] Comment: " << c << std::endl;
    // for (const auto &c : file.get_info())
    //   std::cout << "\t[ply_header] Info: " << c << std::endl;

    // for (const auto &e : file.get_elements()) {
    //   std::cout << "\t[ply_header] element: " << e.name << " (" << e.size <<
    //   ")"
    //             << std::endl;
    //   for (const auto &p : e.properties) {
    //     std::cout << "\t[ply_header] \tproperty: " << p.name << " (type="
    //               << igl::tinyply::PropertyTable[p.propertyType].str << ")";
    //     if (p.isList)
    //       std::cout << " (list_type="
    //                 << igl::tinyply::PropertyTable[p.listType].str << ")";
    //     std::cout << std::endl;
    //   }
    // }

    // Because most people have their own mesh types, tinyply treats parsed data
    // as structured/typed byte buffers. See examples below on how to marry your
    // own application-specific data structures with this one.
    std::shared_ptr<igl::tinyply::PlyData> vertices, normals, colors, texcoords,
        faces, tristrip;

    // The header information can be used to programmatically extract properties
    // on elements known to exist in the header prior to reading the data. For
    // brevity of this sample, properties like vertex position are hard-coded.
    // Also check out the parser from tests.cpp, which shows how to
    // programmatically extract all properties from all elements, which can be
    // helpful to ensure use of of tinyply's little-endian binary fast-path.
    try {
      vertices =
          file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception &e) {
      // std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      normals =
          file.request_properties_from_element("vertex", {"nx", "ny", "nz"});
    } catch (const std::exception &e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors = file.request_properties_from_element(
          "vertex", {"red", "green", "blue", "alpha"});
    } catch (const std::exception &e) {
      // std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors =
          file.request_properties_from_element("vertex", {"r", "g", "b", "a"});
    } catch (const std::exception &e) {
      // std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      texcoords = file.request_properties_from_element("vertex", {"u", "v"});
    } catch (const std::exception &e) {
      // std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // >>> Important Performance Optimization <<<
    // Providing a list size hint (the last argument) is a 2x performance
    // improvement. If you have arbitrary in-the-wild user-imported ply files,
    // set this argument to 0.
    try {
      faces =
          file.request_properties_from_element("face", {"vertex_indices"}, 3);
    } catch (const std::exception &e) {
      // std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // Tristrips must always be read with a list size hint set to 0
    try {
      tristrip = file.request_properties_from_element("tristrips",
                                                      {"vertex_indices"}, 0);
    } catch (const std::exception &e) {
      // std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    manual_timer read_timer;

    read_timer.start();
    file.read(*file_stream);
    read_timer.stop();

    const float parsing_time = static_cast<float>(read_timer.get()) / 1000.f;
    HERMES_INFO("parsing {}mb in {} seconds [{} MBps]", size_mb, parsing_time,
                (size_mb / parsing_time));

    if (vertices)
      HERMES_INFO("Read {} total vertices.", vertices->count);
    if (normals)
      HERMES_INFO("Read {} total vertex normals.", normals->count);
    // if (colors)
    //   std::cout << "\tRead " << colors->count << " total vertex colors "
    //             << std::endl;
    // if (texcoords)
    //   std::cout << "\tRead " << texcoords->count << " total vertex texcoords
    //   "
    //             << std::endl;
    // if (faces)
    //   std::cout << "\tRead " << faces->count << " total faces (triangles) "
    //             << std::endl;
    // if (tristrip)
    //   std::cout << "\tRead "
    //             << (tristrip->buffer.size_bytes() /
    //                 igl::tinyply::PropertyTable[tristrip->t].stride)
    //             << " total indices (tristrip) " << std::endl;
    // if (faces->count)
    //   std::cout << "\tRead " << faces->list_sizes.size()
    //             << " varible-length indices " << std::endl;

    if (vertices->t == igl::tinyply::Type::FLOAT64) { /* as floats ... */
      const size_t numVerticesBytes = vertices->buffer.size_bytes();
      verts.resize(vertices->count);
      std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);
    }
    if (normals->t == igl::tinyply::Type::FLOAT64) { /* as floats ... */
      const size_t numVerticesBytes = normals->buffer.size_bytes();
      norms.resize(normals->count);
      std::memcpy(norms.data(), normals->buffer.get(), numVerticesBytes);
    }

  } catch (const std::exception &e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
  }
  return Result<PointCloud::Ptr>(PointCloud::from(verts, norms));
}

void writePointCloud(std::filesystem::path path, PointCloud::Ptr cloud) {
  auto positions = cloud->positions();
  auto normals = cloud->normals();

  size_t N = cloud->positions().size();

  igl::tinyply::PlyFile file;

  // Add vertex element with 6 properties (pos + normals)
  file.add_properties_to_element("vertex", {"x", "y", "z"},
                                 igl::tinyply::Type::FLOAT64, N,
                                 reinterpret_cast<uint8_t *>(&positions[0]),
                                 igl::tinyply::Type::INVALID, 0);
  file.add_properties_to_element(
      "vertex", {"nx", "ny", "nz"}, igl::tinyply::Type::FLOAT64, N,
      reinterpret_cast<uint8_t *>(&normals[0]), igl::tinyply::Type::INVALID, 0);

  // Write file (binary format recommended for speed/size)
  std::filebuf fb;
  fb.open(path, std::ios::out | std::ios::binary);
  std::ostream outstream(&fb);
  file.write(outstream, true); // true = binary
}

void writeSurface(std::filesystem::path path, PoUSurface::Ptr surface,
                  Scalar voxel_size) {

  auto m = surface->mesh(voxel_size);

  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> V_float =
      m.V.cast<float>();
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> F_uint =
      m.F.cast<uint32_t>();

  HERMES_LOG_VARIABLE(V_float.rows());
  HERMES_LOG_VARIABLE(V_float.cols());

  HERMES_LOG_VARIABLE(F_uint.rows());
  HERMES_LOG_VARIABLE(F_uint.cols());
  HERMES_LOG_VARIABLE(V_float.hasNaN());
  HERMES_LOG_VARIABLE(F_uint.hasNaN());
  HERMES_LOG_VARIABLE(V_float.maxCoeff());
  HERMES_LOG_VARIABLE(V_float.minCoeff());

  HERMES_LOG_VARIABLE(F_uint.maxCoeff());
  HERMES_LOG_VARIABLE(F_uint.minCoeff());
  using namespace igl::tinyply;
  PlyFile mesh;
  // 2. Add Vertices
  mesh.add_properties_to_element(
      "vertex", {"x", "y", "z"}, Type::FLOAT32, V_float.rows(),
      reinterpret_cast<uint8_t *>(V_float.data()), Type::INVALID, 0);

  // 3. Add Faces (Triangles)
  mesh.add_properties_to_element(
      "face", {"vertex_indices"}, Type::UINT32, F_uint.rows(),
      reinterpret_cast<uint8_t *>(F_uint.data()), Type::UINT8, 3);

  // 4. Write
  std::ofstream out_stream(path, std::ios::binary);
  mesh.write(out_stream, true);
}

} // namespace hrbf_surf::io
