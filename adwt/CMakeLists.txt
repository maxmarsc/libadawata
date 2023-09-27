add_library(
  adawata
  ${CMAKE_CURRENT_SOURCE_DIR}/adwt.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/waveform_data.cpp
)

target_include_directories(
  adawata
  PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${ITERTOOLS_INSTALL_DIR}/include
  ${SOXR_INSTALL_DIR}/include
)

target_link_directories(
  adawata
  PUBLIC
  ${SOXR_INSTALL_DIR}/lib
)

target_link_libraries(
  adawata
  PUBLIC
  soxr
)

set_target_properties(
  adawata
  PROPERTIES
  CXX_STANDARD 20
)

add_dependencies(
  adawata
  itertools_project
  soxr_project
)