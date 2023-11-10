# include(FetchContent)

# set(cppitertools_INSTALL_CMAKE_DIR "share" CACHE STRING "" )
# FetchContent_Declare( cppitertools GIT_REPOSITORY
# git@github.com:ryanhaining/cppitertools.git GIT_TAG
# 556fca33235b6ad1fba6e37df104ca23a01a5b45 GIT_SHALLOW ON )

# FetchContent_MakeAvailable(cppitertools)
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
  CMAKE_ARGS
    -Dcppitertools_INSTALL_CMAKE_DIR=share
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}>
)

set(ITERTOOLS_INSTALL_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/itertools/install
    PARENT_SCOPE
)
