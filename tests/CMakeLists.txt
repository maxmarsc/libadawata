include(Catch2)
include(sndfile)
include(argparse)

set(ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/assets")

# ##############################################################################
# sweep generator
# ##############################################################################

add_executable(
  mipmap_transition_generator
  ${CMAKE_CURRENT_SOURCE_DIR}/mipmap_transition_generator.cpp
)

target_include_directories(
  mipmap_transition_generator
  PRIVATE ${PROJECT_SOURCE_DIR} ${SNDFILE_INSTALL_DIR}/include
)

if(ENABLE_CXX20)
  set_target_properties(mipmap_transition_generator PROPERTIES CXX_STANDARD 20)
else()
  set_target_properties(mipmap_transition_generator PROPERTIES CXX_STANDARD 17)
endif()

target_compile_options(
  mipmap_transition_generator
  PRIVATE -Wall -Wextra
)

target_link_directories(
  mipmap_transition_generator PRIVATE ${SNDFILE_INSTALL_DIR}/lib
)

target_link_libraries(
  mipmap_transition_generator argparse adawata sndfile
)

target_compile_definitions(
  mipmap_transition_generator
  PRIVATE ASSETS_DIR="${ASSETS_DIR}"
)

target_link_options(
  mipmap_transition_generator PUBLIC $<$<BOOL:${USE_ASAN}>:-fsanitize=address
  -static-libasan>
)

add_dependencies(mipmap_transition_generator libsndfile_project)

# ##############################################################################
# sweep generator
# ##############################################################################

add_executable(
  sweep_generator
  ${CMAKE_CURRENT_SOURCE_DIR}/sweep_generator.cpp
)

target_include_directories(
  sweep_generator
  PRIVATE ${PROJECT_SOURCE_DIR} ${SNDFILE_INSTALL_DIR}/include
)

if(ENABLE_CXX20)
  set_target_properties(sweep_generator PROPERTIES CXX_STANDARD 20)
else()
  set_target_properties(sweep_generator PROPERTIES CXX_STANDARD 17)
endif()

target_compile_options(
  sweep_generator
  PRIVATE -Wall -Wextra
)

target_link_directories(
  sweep_generator PRIVATE ${SNDFILE_INSTALL_DIR}/lib
)

target_link_libraries(
  sweep_generator argparse adawata sndfile
)

target_compile_definitions(
  sweep_generator
  PRIVATE ASSETS_DIR="${ASSETS_DIR}"
)

target_link_options(
  sweep_generator PUBLIC $<$<BOOL:${USE_ASAN}>:-fsanitize=address
  -static-libasan>
)

add_dependencies(sweep_generator libsndfile_project)

# ##############################################################################
# Testing
# ##############################################################################
add_executable(
  libadawata_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/wavetable_data_tests.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/oscillator_tests.cpp
)

target_include_directories(
  libadawata_tests
  PRIVATE ${PROJECT_SOURCE_DIR} ${SNDFILE_INSTALL_DIR}/include
)

if(ENABLE_CXX20)
  set_target_properties(libadawata_tests PROPERTIES CXX_STANDARD 20)
else()
  set_target_properties(libadawata_tests PROPERTIES CXX_STANDARD 17)
endif()

target_compile_options(
  libadawata_tests
  PRIVATE -Wall -Wextra
)

target_link_directories(
  libadawata_tests PRIVATE ${SNDFILE_INSTALL_DIR}/lib
)

target_link_libraries(
  libadawata_tests Catch2::Catch2WithMain adawata sndfile
)

target_compile_definitions(
  libadawata_tests
  PRIVATE ASSETS_DIR="${ASSETS_DIR}"
)

target_link_options(
  libadawata_tests PUBLIC $<$<BOOL:${USE_ASAN}>:-fsanitize=address
  -static-libasan>
)

add_dependencies(libadawata_tests libsndfile_project)
add_test(NAME libadawata_tests COMMAND libadawata_tests)

# ##############################################################################
# Benchmarks
# ##############################################################################
if(BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()
