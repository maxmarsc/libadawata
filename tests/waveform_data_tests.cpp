/*
* waveform_data_tests.cpp
* Created by maxmarsc, 27/09/2023
*/

#include <cmath>

#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <cppitertools/range.hpp>

#include "adwt/waveform_data.hpp"

namespace tests {

TEST_CASE("Valid build") {
  const auto waveform_len = GENERATE(1024, 2048, 4096);
  // const auto num_waveforms = GENERATE(1, 2, 64, 256);
  const auto num_waveforms = GENERATE(1, 2, 8);
  // const auto num_waveforms = GENERATE(1);
  // const auto samplerate = GENERATE(16000, 32000, 44100, 48000, 88200, 96000);
  const auto samplerate = 44100;

  auto waveforms = std::vector<float>(waveform_len * num_waveforms);
  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

  INFO("waveform_len : " << waveform_len);
  INFO("num_waveforms : " << num_waveforms);
  INFO("samplerate : " << samplerate);
  REQUIRE(waveform_data.get() != nullptr);
  CHECK(waveform_data->numWaveforms() == num_waveforms);

  // Check the number of mipmap tables match the expectations
  const auto expected_mipmap_tables = std::log2(waveform_len) - 1;
  REQUIRE(waveform_data->numMipMapTables() == expected_mipmap_tables);

  // Check the access and size of each sub table in the data
  for (auto mipmap_idx : iter::range(waveform_data->numMipMapTables())) {
    const auto expected_size =
        static_cast<std::size_t>(waveform_len) / (1 << mipmap_idx);

    auto phase_span = waveform_data->phases(mipmap_idx);
    CHECK(phase_span.size() == expected_size + 1);

    for (auto waveform_idx : iter::range(waveform_data->numWaveforms())) {
      auto m_span  = waveform_data->m(waveform_idx, mipmap_idx);
      auto q_span  = waveform_data->q(waveform_idx, mipmap_idx);
      auto md_span = waveform_data->mDiff(waveform_idx, mipmap_idx);
      auto qd_span = waveform_data->qDiff(waveform_idx, mipmap_idx);

      CHECK(m_span.size() == expected_size);
      CHECK(q_span.size() == expected_size);
      CHECK(md_span.size() == expected_size);
      CHECK(qd_span.size() == expected_size);
    }
  }

  // Check mip map index function
  auto rng = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 0.49999F, Catch::getSeed());
  auto count = 1000;
  while (--count) {
    rng.next();
    const auto abs_phase_diff = rng.get();

    auto&& [crt_idx, crt_weight, nxt_idx, nxt_weight] =
        waveform_data->findMipMapIndices(abs_phase_diff);

    INFO("phase_diff : " << abs_phase_diff);
    REQUIRE(crt_weight + nxt_weight == 1.0F);
    REQUIRE(crt_idx >= 0);
    REQUIRE(crt_idx < waveform_data->numMipMapTables());
    REQUIRE(crt_idx + 1 == nxt_idx);
  }
}

TEST_CASE("Invalid samplerate") {
  const auto waveform_len  = GENERATE(1024, 2048, 4096);
  const auto num_waveforms = GENERATE(1, 2, 64, 256);
  const auto samplerate    = GENERATE(-16000, 0);

  auto waveforms = std::vector<float>(waveform_len * num_waveforms);
  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

  REQUIRE(waveform_data.get() == nullptr);
}

TEST_CASE("Invalid waveform span") {
  const auto waveform_len  = GENERATE(1024, 2048, 4096);
  const auto num_waveforms = GENERATE(1, 2, 64, 256);
  const auto samplerate    = GENERATE(16000, 32000, 44100, 48000, 88200, 96000);
  const auto len_offset    = GENERATE(-2, -1, 1, 2);

  auto waveforms =
      std::vector<float>(waveform_len * num_waveforms + len_offset);
  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

  INFO("waveforms : " << waveform_len * num_waveforms + len_offset);
  INFO("num_waveforms : " << num_waveforms);
  REQUIRE(waveform_data.get() == nullptr);
}

TEST_CASE("Invalid num_waveforms") {
  const auto waveform_len  = GENERATE(1024, 2048, 4096);
  const auto num_waveforms = GENERATE(0, -1, -64);
  const auto samplerate    = GENERATE(16000, 32000, 44100, 48000, 88200, 96000);

  auto waveforms = std::vector<float>(waveform_len * 64);
  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

  REQUIRE(waveform_data.get() == nullptr);
}

}  // namespace tests