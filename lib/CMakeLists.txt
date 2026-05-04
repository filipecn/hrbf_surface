# ##############################################################################
# hrbf_surf lib                                                                 ##
# ##############################################################################
set(HRBF_SURF_HEADERS
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/io.h
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/hrbf.h
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/point_cloud.h
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/pou_surface.h
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/result.h
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/debug.h
)

set(HRBF_SURF_SOURCES
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/io.cpp
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/point_cloud.cpp
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/pou_surface.cpp
  ${HRBF_SURF_SOURCE_DIR}/hrbf_surf/hrbf.cpp
)

add_library(hrbf_surf STATIC ${HRBF_SURF_SOURCES} ${HRBF_SURF_HEADERS})
add_dependencies(hrbf_surf hermes)
target_include_directories(
  hrbf_surf PRIVATE  
  ${DEPS_INCLUDE_DIRS}
  ${HRBF_SURF_SOURCE_DIR}
)
target_link_libraries(hrbf_surf PRIVATE DEPS)

if(HRBF_SURF_INCLUDE_DEBUG_TRAITS)
  target_compile_definitions(hrbf_surf PUBLIC HRBF_SURF_INCLUDE_DEBUG_TRAITS)
  target_compile_definitions(hrbf_surf PUBLIC HERMES_INCLUDE_DEBUG_TRAITS)
endif(HRBF_SURF_INCLUDE_DEBUG_TRAITS)

  set_target_properties(
    hrbf_surf
    PROPERTIES CXX_STANDART 23
    CXX_STANDARD_REQUIRED YES
    LINKER_LANGUAGE CXX
  )

 if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(
      hrbf_surf
      PUBLIC
      # cuda
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CUDA>>,-Xcompiler=-Wextra,>"
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CUDA>>,-Xcompiler=-Wall,>"
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CUDA>>,-Xcompiler=-Werror,>"
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CUDA>>,-Xcompiler=-O0,>"
      # CXX
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CXX>>,-Wextra,>"
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CXX>>,-Wall,>"
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CXX>>,-Werror,>"
      "$<IF:$<AND:$<CONFIG:DEBUG>,$<COMPILE_LANGUAGE:CXX>>,-O0,>"
      # common
      "$<$<CONFIG:Debug>:-g;-pg;-fprofile-arcs;-ftest-coverage;--coverage;-fPIC>"
      "$<$<CONFIG:Release>:-O3;-fPIC;>")
elseif(MSVC)
    target_compile_options(
      hrbf_surf 
      PUBLIC 
      "$<$<CONFIG:Debug>:/Zc:__cplusplus;/Zc:preprocessor;/EHsc;/MDd>"
      "$<$<CONFIG:Release>:/Zc:__cplusplus;/Zc:preprocessor/EHsc;/MD>")
endif(CMAKE_COMPILER_IS_GNUCXX)


# ##############################################################################
# python                                                                      ##
# ##############################################################################
pybind11_add_module(hrbf_surface_py hrbf_surface_py.cpp)

add_dependencies(hrbf_surface_py hermes)
target_link_libraries(hrbf_surface_py PUBLIC 
 DEPS
 hrbf_surf
)

target_include_directories(hrbf_surface_py PUBLIC 
 ${DEPS_INCLUDE_DIRS}
 ${HRBF_SURF_SOURCE_DIR})

if(HRBF_SURF_INCLUDE_DEBUG_TRAITS)
 target_compile_definitions(hrbf_surface_py PUBLIC hrbf_surf_INCLUDE_DEBUG_TRAITS)
 target_compile_definitions(hrbf_surface_py PUBLIC HERMES_INCLUDE_DEBUG_TRAITS)
endif(HRBF_SURF_INCLUDE_DEBUG_TRAITS)
