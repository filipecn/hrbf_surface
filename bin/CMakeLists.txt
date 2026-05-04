add_executable(hrbf_surface main.cpp)

target_include_directories(hrbf_surface PUBLIC 
${HRBF_SURF_SOURCE_DIR}
${DEPS_INCLUDE_DIRS}
)

setup_target(hrbf_surface)

if(HRBF_SURF_INCLUDE_DEBUG_TRAITS)
  target_compile_definitions(hrbf_surface PUBLIC HRBF_SURF_INCLUDE_DEBUG_TRAITS)
  target_compile_definitions(hrbf_surface PUBLIC HERMES_INCLUDE_DEBUG_TRAITS)
endif(HRBF_SURF_INCLUDE_DEBUG_TRAITS)

target_link_libraries(hrbf_surface PRIVATE 
DEPS
hrbf_surf
)


