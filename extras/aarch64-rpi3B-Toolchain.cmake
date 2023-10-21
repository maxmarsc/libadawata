# targets
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(VERSION "10.2.1")

# misc settings
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set(TOOLCHAIN_ROOT "/usr/local/aarch64-none-linux-gnu")
set(CMAKE_SYSROOT "${TOOLCHAIN_ROOT}/aarch64-none-linux-gnu/libc")

# find paths
set(CMAKE_FIND_ROOT_PATH "${TOOLCHAIN_ROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# compiler settings
set(CMAKE_C_COMPILER
    "${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-gcc"
    CACHE INTERNAL ""
)
set(CMAKE_CXX_COMPILER
    "${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-g++"
    CACHE INTERNAL ""
)

set(CMAKE_C_COMPILER_EXTERNAL_TOOLCHAIN "${TOOLCHAIN_ROOT}/lib/gcc")
set(CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN "${TOOLCHAIN_ROOT}/lib/gcc")

# CPU tuning : the RPI3B+ uses a Cortex a53
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -mtune=cortex-a53"
    CACHE STRING ""
)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -mtune=cortex-a53"
    CACHE STRING ""
)

# Qemu emulation setup
set(CMAKE_CROSSCOMPILING_EMULATOR
    "qemu-aarch64-static;-L;${CMAKE_SYSROOT}"
    CACHE FILEPATH "Path to the emulator for the target system."
)

# Include settings
set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
    ${TOOLCHAIN_ROOT}/aarch64-none-linux-gnu/include/c++/${VERSION}
    ${TOOLCHAIN_ROOT}/aarch64-none-linux-gnu/libc/usr/include
    ${TOOLCHAIN_ROOT}/include
    ${TOOLCHAIN_ROOT}/aarch64-none-linux-gnu/include/c++/${VERSION}/aarch64-none-linux-gnu
    ${TOOLCHAIN_ROOT}/lib/gcc/aarch64-none-linux-gnu/${VERSION}/include
)

set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES
    ${TOOLCHAIN_ROOT}/aarch64-none-linux-gnu/libc/usr/include
)

set(CMAKE_CXX_STANDARD_LIBRARIES "-lpthread -lgomp")

set(CMAKE_C_STANDARD_LIBRARIES "-lpthread -lgomp")
