# ##############################################################################
# CMAKE                                                                        #
# ##############################################################################
cmake_minimum_required(VERSION 3.12..3.30 FATAL_ERROR)

project(
  HRBF_SURF
  VERSION 0.01
  DESCRIPTION "Surface Reconstruction"
  LANGUAGES C CXX)

cmake_path(SET HRBF_SURF_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib)
cmake_path(SET HRBF_SURF_EXAMPLES_DIR ${CMAKE_CURRENT_LIST_DIR}/examples)

# cmake print functions
include(CMakePrintHelpers)

# ##############################################################################
# OPTIONS                                                                      #
# ##############################################################################
option(HRBF_SURF_BUILD_ALL "build all targets" OFF)
option(HRBF_SURF_BUILD_TESTS "build tests" OFF)
option(HRBF_SURF_BUILD_EXAMPLES "build examples" OFF)
option(HRBF_SURF_BUILD_DOCS "build library documentation" OFF)
option(HRBF_SURF_INCLUDE_DEBUG_TRAITS "enable naiades::to_string methods" ON)

# ##############################################################################
# COMPILATION                                                                  #
# ##############################################################################
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
include(ConfigCompilerFlags)
include(ExternalPrograms)

# ##############################################################################
# DEPENDENCIES                                                                 #
# ##############################################################################
add_subdirectory(ext)

# ##############################################################################
# ENGINE                                                                       #
# ##############################################################################
add_subdirectory(lib)

# ##############################################################################
# TESTS                                                                        #
# ##############################################################################
if (HRBF_SURF_BUILD_TESTS)
#  add_subdirectory(tests)
endif (HRBF_SURF_BUILD_TESTS)

# ##############################################################################
# COMPILATION/RUN SANITY TEST                                                  #
# ##############################################################################
# try_compile(COMPILATION_TEST_RESULT bindir SOURCES source.cpp)
# try_run(RUN_TEST_RESULT bindr SOURCES source.cpp)

add_subdirectory(bin)

if (HRBF_SURF_BUILD_EXAMPLES)
  # add_subdirectory(examples)
endif (HRBF_SURF_BUILD_EXAMPLES)

# ##############################################################################
#                                                                DOCUMENTATION #
# ##############################################################################
if(HRBF_SURF_BUILD_DOCS)
  # include(GenerateDocs)
endif(HRBF_SURF_BUILD_DOCS)
