include(FetchContent)
# include(curl)

set(JUCE_USE_CURL OFF)
FetchContent_Declare(
  juce
  GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
  GIT_PROGRESS TRUE
  GIT_SHALLOW TRUE
)
set(JUCE_MODULES_ONLY ON)
FetchContent_MakeAvailable(juce)

add_library(
  ovs_oscillator
  ${CMAKE_CURRENT_SOURCE_DIR}/ovs_oscillator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/downsampler_juce.cpp
)

target_include_directories(
  ovs_oscillator
  PUBLIC ${PROJECT_SOURCE_DIR} ${ITERTOOLS_INSTALL_DIR}/include
)

target_link_directories(
  ovs_oscillator PUBLIC
)

target_link_libraries(
  ovs_oscillator
  PUBLIC adawata juce::juce_recommended_config_flags
         # juce::juce_recommended_lto_flags
         juce::juce_recommended_warning_flags juce::juce_dsp
)

if(ENABLE_CXX20)
  set_target_properties(ovs_oscillator PROPERTIES CXX_STANDARD 20)
else()
  set_target_properties(ovs_oscillator PROPERTIES CXX_STANDARD 17)
endif()

target_compile_options(
  ovs_oscillator
  PRIVATE -Wall -Wextra -ffast-math $<$<BOOL:${USE_ASAN}>:-fsanitize=address>
)

target_compile_definitions(
  ovs_oscillator
  PUBLIC JUCE_USE_CURL=0
)

target_link_options(
  ovs_oscillator PUBLIC $<$<BOOL:${USE_ASAN}>:-fsanitize=address
  -static-libasan>
)

add_dependencies(ovs_oscillator itertools_project)
