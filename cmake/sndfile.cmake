# simplify variable expansion
# cmake_policy(SET CMP0053 NEW)
# cmake_policy(SET CMP0010 NEW)
# cmake_policy(SET CMP0091 NEW)

set(SNDFILE_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/sndfile/install)
include(ExternalProject)
ExternalProject_Add(
  libsndfile_project
  GIT_REPOSITORY git@github.com:libsndfile/libsndfile.git
  GIT_TAG "2ed38b57c5838ba6b7c8e726a62855845bd42ec5"
  GIT_SHALLOW ON
  INSTALL_DIR ${SNDFILE_INSTALL_DIR}
  LOG_CONFIGURE ON
  LOG_BUILD ON
  LOG_UPDATE ON
  LOG_INSTALL ON
  LOG_OUTPUT_ON_FAILURE ON
  CMAKE_ARGS -DCMP0091=NEW
             -DBUILD_PROGRAMS=OFF
             -DBUILD_EXAMPLES=OFF
             -DBUILD_TESTING=OFF
             -DENABLE_EXTERNAL_LIBS=OFF
             -DENABLE_MPEG=OFF
             ${CROSS_COMPILE_CMAKE_ARGS}
  INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR> --prefix
                  <INSTALL_DIR>
)

set(SNDFILE_INSTALL_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/sndfile/install
    PARENT_SCOPE
)
