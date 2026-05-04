include(ExternalProject)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

set(DEPS_INCLUDE_DIRS "")

add_library(DEPS INTERFACE)

include(hermes)
include(nanoflann)
include(pybind11)
include(libigl)
include(eigen)
include(simple_svg)
include(tinyply)

list(APPEND DEPS_INCLUDE_DIRS ";${HERMES_INCLUDE_DIR}")
list(APPEND DEPS_INCLUDE_DIRS ";${NANOFLANN_INCLUDE_DIR}")
list(APPEND DEPS_INCLUDE_DIRS ";${LIBIGL_INCLUDE_DIR}")
list(APPEND DEPS_INCLUDE_DIRS ";${SIMPLE_SVG_INCLUDE_DIR}")
list(APPEND DEPS_INCLUDE_DIRS ";${TINYPLY_INCLUDE_DIR}")

target_link_libraries(DEPS INTERFACE 
  hermes
  tinyply
  Eigen3::Eigen
  igl::core
  igl_restricted::triangle
)

if (HRBF_SURF_BUILD_TESTS)  
  include(catch2)
endif (HRBF_SURF_BUILD_TESTS)

set(DEPS_INCLUDE_DIRS ${DEPS_INCLUDE_DIRS} CACHE STRING "" FORCE)
