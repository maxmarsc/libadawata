cmake_minimum_required(VERSION 3.16)

project(libadawata VERSION 0.0.1)

cmake_policy(SET CMP0077 NEW)

# cmake_policy(SET CMP0053 NEW) cmake_policy(SET CMP0010 NEW) cmake_policy(SET
# CMP0135 NEW) cmake_policy(SET CMP0091 NEW)
option(BUILD_TESTING "Enable the build of the tests" On)
option(BUILD_BENCHMARKS
       "Enable the build of the benchmarking binaries. Will force BUILD_TESTING"
       Off
)
option(
  ENABLE_CXX20
  "Enable the use of C++20, otherwise only C++17 is supported and gsl::span will be used"
  On
)

# Debug options
option(USE_TRACY "Enable tracy profiling of benchmark binaries" Off)
option(USE_ASAN "Use address sanitize" Off)

set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES Off)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

message(STATUS "CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE}")

# ##############################################################################
# Misc
# ##############################################################################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# ##############################################################################
# Tracy
# ##############################################################################
if(USE_TRACY)
  add_compile_definitions(TRACY_ENABLE)
  add_compile_definitions(TRACY_NO_EXIT)
endif()

# ##############################################################################
# Sources
# ##############################################################################
add_subdirectory(cmake)
add_subdirectory(adwt)

# ##############################################################################
# Testing
# ##############################################################################
if(BUILD_BENCHMARKS)
  set(BUILD_TESTING ON)
endif()
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  # include(CTest)
  enable_testing()
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  add_subdirectory(tests)
  set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 0)
endif()

# #
# ##############################################################################
# # Benchmarks #
# ##############################################################################
# if(BUILD_BENCHMARKS) add_subdirectory(benchmarks) endif()
