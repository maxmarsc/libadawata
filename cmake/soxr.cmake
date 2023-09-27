include(ExternalProject)
set(SOXR_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/soxr/install)
ExternalProject_Add(
  soxr_project
  GIT_REPOSITORY  git@github.com:chirlu/soxr.git
  GIT_TAG         0.1.3
  GIT_SHALLOW     ON
  INSTALL_DIR     ${SOXR_INSTALL_DIR}
  LOG_CONFIGURE   ON
  LOG_BUILD       ON
  LOG_INSTALL     ON
  LOG_OUTPUT_ON_FAILURE ON
  CMAKE_ARGS  -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> 
              -DBUILD_TESTS=OFF 
              -DWITH_LSR_BINDINGS=OFF
)

set(SOXR_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/soxr/install PARENT_SCOPE)
