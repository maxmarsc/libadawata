include(Catch2)

add_executable(
  libadawata_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/waveform_data_tests.cpp
)

target_include_directories(
  libadawata_tests
  PRIVATE ${PROJECT_SOURCE_DIR}
)

set_target_properties(
  libadawata_tests
  PROPERTIES CXX_STANDARD 20
)

target_compile_options(
  libadawata_tests
  PRIVATE -Wall -Wextra
)

target_link_libraries(
  libadawata_tests Catch2::Catch2WithMain adawata
)

target_compile_definitions(
  libadawata_tests
  PRIVATE
)

add_test(NAME libadawata_tests COMMAND libadawata_tests)
