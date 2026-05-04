include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.6.0
)

set(LIBIGL_RESTRICTED_TRIANGLE ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(libigl)

set(LIBIGL_INCLUDE_DIR ${libigl_SOURCE_DIR}/include CACHE STRING "" FORCE)
