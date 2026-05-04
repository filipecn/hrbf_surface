include(FetchContent)

FetchContent_Declare(
  simple_svg 
  GIT_REPOSITORY https://github.com/adishavit/simple-svg.git
  GIT_TAG v1.0.1)

set(BUILD_EXAMPLES  OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING  OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(simple_svg)

set(SIMPLE_SVG_INCLUDE_DIR ${simple_svg_SOURCE_DIR} CACHE STRING "" FORCE)

