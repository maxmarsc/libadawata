set(ITERTOOLS_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/itertools/install)
include(ExternalProject)
ExternalProject_Add(
  itertools_project
  GIT_REPOSITORY git@github.com:ryanhaining/cppitertools.git
  GIT_TAG v2.1
  GIT_SHALLOW ON
  INSTALL_DIR ${ITERTOOLS_INSTALL_DIR}
  LOG_CONFIGURE ON
  LOG_INSTALL ON
  CMAKE_ARGS -Dcppitertools_INSTALL_CMAKE_DIR=share
             -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
)

set(ITERTOOLS_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/itertools/install PARENT_SCOPE)