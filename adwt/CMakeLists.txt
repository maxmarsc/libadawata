include(xsimd)
include(Tracy)

add_library(
  adawata
  ${CMAKE_CURRENT_SOURCE_DIR}/adwt.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/waveform_data.cpp
)

target_include_directories(
  adawata
  PUBLIC ${PROJECT_SOURCE_DIR} ${ITERTOOLS_INSTALL_DIR}/include
         ${SOXR_INSTALL_DIR}/include ${TRACY_INSTALL_DIR}/include
)

target_link_directories(
  adawata PUBLIC ${SOXR_INSTALL_DIR}/lib ${TRACY_INSTALL_DIR}/lib
)

target_link_libraries(
  adawata
  PUBLIC soxr
         xsimd
         TracyClient
         $<$<BOOL:${USE_TRACY}>:pthread>
         $<$<BOOL:${USE_TRACY}>:dl>
         # $<$<BOOL:${USE_ASAN}>:asan>
)

set_target_properties(
  adawata
  PROPERTIES CXX_STANDARD 20
)

target_compile_options(
  adawata
  PUBLIC -ffast-math $<$<BOOL:${USE_ASAN}>:-fsanitize=address>
  # -fopt-info-vec-optimized
  # -fopt-info-vec-missed
)

target_link_options(
  adawata PUBLIC $<$<BOOL:${USE_ASAN}>:-fsanitize=address -static-libasan>
)

add_dependencies(adawata itertools_project soxr_project tracy_project)
