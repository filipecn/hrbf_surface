include(FetchContent)
FetchContent_Declare(
  tinyply
  GIT_REPOSITORY https://github.com/ddiakopoulos/tinyply.git
  GIT_TAG 3.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)

FetchContent_MakeAvailable(tinyply)

set(TINYPLY_INCLUDE_DIR ${tinyply_SOURCE_DIR}/source CACHE STRING "" FORCE)