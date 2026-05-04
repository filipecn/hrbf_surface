include(FetchContent)
FetchContent_Declare(
    nanoflann
    GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
    GIT_TAG v1.9.0
)

set(NANOFLANN_BUILD_EXAMPLES  OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_TESTS   OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(nanoflann)

set(NANOFLANN_INCLUDE_DIR ${nanoflann_SOURCE_DIR}/include CACHE STRING "" FORCE)
