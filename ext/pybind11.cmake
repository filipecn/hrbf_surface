include(FetchContent)

set(PYBIND11_PYTHON_VERSION 3.14.3 CACHE STRING "")

FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG        v3.0.1
)

FetchContent_MakeAvailable(pybind11)
