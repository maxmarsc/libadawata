cmake_minimum_required(VERSION 3.16)
include(ExternalProject)

# ##############################################################################
# benchmark
# ##############################################################################
set(BENCHMARK_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/benchmark/install)
ExternalProject_Add(
  benchmark_project
  GIT_REPOSITORY https://github.com/google/benchmark
  GIT_TAG v1.8.0
  GIT_SHALLOW ON
  INSTALL_DIR ${BENCHMARK_INSTALL_DIR}
  LOG_CONFIGURE ON
  LOG_BUILD ON
  LOG_UPDATE ON
  LOG_INSTALL ON
  LOG_OUTPUT_ON_FAILURE ON
  CMAKE_ARGS
    -D
    BENCHMARK_ENABLE_TESTING=OFF
    -D
    CMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -D
    CMAKE_BUILD_TYPE:STRING=Release
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}>
)

set(BENCHMARK_INSTALL_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/benchmark/install
    PARENT_SCOPE
)
