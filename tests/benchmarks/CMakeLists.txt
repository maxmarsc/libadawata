include(Gbenchmark)

add_library(
  utils_benchmarks
  ${CMAKE_CURRENT_SOURCE_DIR}/utils_benchmarks.cpp
)

target_include_directories(
  utils_benchmarks
  PUBLIC ${PROJECT_SOURCE_DIR} ${ITERTOOLS_INSTALL_DIR}/include
)

set_target_properties(
  utils_benchmarks
  PROPERTIES CXX_STANDARD 20
)

target_compile_options(
  utils_benchmarks
  PRIVATE -Wall -Wextra
)

add_dependencies(utils_benchmarks itertools_project)

# ##############################################################################
# libadawata
# ##############################################################################
add_executable(
  adwt_benchmark
  ${CMAKE_CURRENT_SOURCE_DIR}/adwt_benchmark.cpp
)

target_include_directories(
  adwt_benchmark
  PRIVATE ${PROJECT_SOURCE_DIR} ${BENCHMARK_INSTALL_DIR}/include
          ${SNDFILE_INSTALL_DIR}/include
)

set_target_properties(
  adwt_benchmark
  PROPERTIES CXX_STANDARD 20
)

target_compile_options(
  adwt_benchmark
  PRIVATE -Wall -Wextra
)

target_compile_definitions(
  adwt_benchmark
  PRIVATE ASSETS_DIR="${ASSETS_DIR}"
)

target_link_directories(
  adwt_benchmark PRIVATE ${BENCHMARK_INSTALL_DIR}/lib
  ${SNDFILE_INSTALL_DIR}/lib
)

target_link_libraries(
  adwt_benchmark
  benchmark
  benchmark_main
  utils_benchmarks
  adawata
  sndfile
)

add_dependencies(adwt_benchmark benchmark_project libsndfile_project)

add_executable(
  adwt_profiler
  ${CMAKE_CURRENT_SOURCE_DIR}/adwt_profile.cpp
)

target_include_directories(
  adwt_profiler
  PRIVATE ${PROJECT_SOURCE_DIR} ${SNDFILE_INSTALL_DIR}/include
)

set_target_properties(
  adwt_profiler
  PROPERTIES CXX_STANDARD 20
)

target_compile_options(
  adwt_profiler
  PRIVATE -Wall -Wextra
)

target_compile_definitions(
  adwt_profiler
  PRIVATE ASSETS_DIR="${ASSETS_DIR}"
)

target_link_directories(
  adwt_profiler PRIVATE ${SNDFILE_INSTALL_DIR}/lib
)

target_link_libraries(
  adwt_profiler utils_benchmarks adawata sndfile
)

add_dependencies(adwt_benchmark benchmark_project libsndfile_project)

add_subdirectory(source)

# ##############################################################################
# juce-resampler
# ##############################################################################
add_executable(
  juce_benchmark
  ${CMAKE_CURRENT_SOURCE_DIR}/juce_benchmark.cpp
)

target_include_directories(
  juce_benchmark
  PRIVATE ${PROJECT_SOURCE_DIR} ${BENCHMARK_INSTALL_DIR}/include
          ${SNDFILE_INSTALL_DIR}/include
)

set_target_properties(
  juce_benchmark
  PROPERTIES CXX_STANDARD 20
)

target_compile_options(
  juce_benchmark
  PRIVATE -Wall -Wextra
)

target_compile_definitions(
  juce_benchmark
  PRIVATE ASSETS_DIR="${ASSETS_DIR}"
)

target_link_directories(
  juce_benchmark PRIVATE ${BENCHMARK_INSTALL_DIR}/lib
  ${SNDFILE_INSTALL_DIR}/lib
)

target_link_libraries(
  juce_benchmark
  benchmark
  benchmark_main
  utils_benchmarks
  adawata
  sndfile
  ovs_oscillator
)

add_dependencies(juce_benchmark benchmark_project libsndfile_project)
