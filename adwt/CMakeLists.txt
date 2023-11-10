include(xsimd)

if(NOT ENABLE_CXX20)
  include(gsl)
endif()

add_library(
  adawata
  ${CMAKE_CURRENT_SOURCE_DIR}/adwt.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/waveform_data.cpp
)

# add_library(adawata::adawata ALIAS adawata)

target_include_directories(
  adawata
  PUBLIC ${PROJECT_SOURCE_DIR} ${ITERTOOLS_INSTALL_DIR}/include
)

target_link_libraries(
  adawata
  PUBLIC xsimd $<$<NOT:$<BOOL:${ENABLE_CXX20}>>:Microsoft.GSL::GSL>
         SampleRate::samplerate
)

if(ENABLE_CXX20)
  set_target_properties(adawata PROPERTIES CXX_STANDARD 20)
else()
  set_target_properties(adawata PROPERTIES CXX_STANDARD 17)
endif()

target_compile_options(
  adawata
  PUBLIC -ffast-math $<$<BOOL:${USE_ASAN}>:-fsanitize=address>
  # -fopt-info-vec-optimized -fopt-info-vec-missed
)

target_compile_definitions(
  adawata
  PUBLIC $<$<BOOL:${ENABLE_CXX20}>:ADWT_ENABLE_CXX20>
)

target_link_options(
  adawata PUBLIC $<$<BOOL:${USE_ASAN}>:-fsanitize=address -static-libasan>
)

add_dependencies(adawata itertools_project)

# ##############################################################################
# Installation
# ##############################################################################

# It seems this is an old-schoo

install(
  TARGETS adawata
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/adwt.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/direction.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/filter_type.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/maths.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/span.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/waveform_data.hpp
  DESTINATION include/adwt
)

install(FILES ${ITERTOOLS_INSTALL_DIR}/include/cppitertools/range.hpp
              ${ITERTOOLS_INSTALL_DIR}/include/cppitertools/zip.hpp
        DESTINATION include/cppitertools
)

if(NOT ENABLE_CXX20)
  install(
    FILES ${GSL_SOURCE_DIR}/include/gsl/span ${GSL_SOURCE_DIR}/include/gsl/byte
          ${GSL_SOURCE_DIR}/include/gsl/util
          ${GSL_SOURCE_DIR}/include/gsl/assert DESTINATION include/gsl
  )
endif()
