/*
* waveform_data_tests.cpp
* Created by maxmarsc, 27/09/2023
*/

#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>
#include <catch2/generators/catch_generators_random.hpp>

#include "adwt/waveform_data.hpp"

namespace tests {

TEST_CASE("Valid build") {
  const auto waveform_len  = GENERATE(1024, 2048, 4096);
  const auto num_waveforms = GENERATE(1, 2, 64, 256);
  const auto samplerate    = GENERATE(16000, 24000, 44100, 48000, 88200, 96000);

  auto waveforms = std::vector<float>(waveform_len * num_waveforms);
  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

  CHECK(waveform_data.get() != nullptr);
}

}  // namespace tests