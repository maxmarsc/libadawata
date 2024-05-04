# targets
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(triple x86_64-linux-gnu)

# compiler settings
set(CMAKE_C_COMPILER
    "clang"
    CACHE INTERNAL ""
)
set(CMAKE_CXX_COMPILER
    "clang++"
    CACHE INTERNAL ""
)

set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -march=native"
    CACHE STRING ""
)
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -march=native"
    CACHE STRING ""
)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=cascadelake" CACHE STRING "" )
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=cascadelake" CACHE STRING "" )
