/*
* wavetable_data_tests.cpp
* Created by maxmarsc, 27/09/2023
*
* This work is licensed under the MIT License
*
* Copyright (c) 2023 Maxime Coutant
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include <cmath>

#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <cppitertools/range.hpp>

#include "adwt/wavetable_data.hpp"

namespace tests {

TEST_CASE("Valid build") {
  const auto waveform_len = GENERATE(1024, 2048, 4096);
  // const auto num_waveforms = GENERATE(1, 2, 64, 256);
  const auto num_waveforms = GENERATE(1, 2, 8);
  // const auto num_waveforms = GENERATE(1);
  // const auto samplerate = GENERATE(16000, 32000, 44100, 48000, 88200, 96000);
  const auto samplerate = 44100;

  auto waveforms = std::vector<float>(waveform_len * num_waveforms);
  auto wavetable_data =
      adwt::WavetableData::build(waveforms, num_waveforms, samplerate);

  INFO("waveform_len : " << waveform_len);
  INFO("num_waveforms : " << num_waveforms);
  INFO("samplerate : " << samplerate);
  REQUIRE(wavetable_data.get() != nullptr);
  CHECK(wavetable_data->numWaveforms() == num_waveforms);

  // Check the number of mipmap tables match the expectations
  const auto expected_mipmap_tables = std::log2(waveform_len) - 1;
  REQUIRE(wavetable_data->numMipMapTables() == expected_mipmap_tables);

  // Check the access and size of each sub table in the data
  for (auto mipmap_idx : iter::range(wavetable_data->numMipMapTables())) {
    const auto expected_size =
        static_cast<std::size_t>(waveform_len) / (1 << mipmap_idx);

    auto phase_span = wavetable_data->phases(mipmap_idx);
    CHECK(phase_span.size() == expected_size + 1);

    for (auto waveform_idx : iter::range(wavetable_data->numWaveforms())) {
      auto m_span  = wavetable_data->m(waveform_idx, mipmap_idx);
      auto q_span  = wavetable_data->q(waveform_idx, mipmap_idx);
      auto md_span = wavetable_data->mDiff(waveform_idx, mipmap_idx);
      auto qd_span = wavetable_data->qDiff(waveform_idx, mipmap_idx);

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
        wavetable_data->findMipMapIndices(abs_phase_diff);

    INFO("phase_diff : " << abs_phase_diff);
    REQUIRE(crt_weight + nxt_weight == 1.0F);
    REQUIRE(crt_idx >= 0);
    REQUIRE(crt_idx < wavetable_data->numMipMapTables());
    REQUIRE(crt_idx + 1 == nxt_idx);
  }
}

TEST_CASE("Invalid samplerate") {
  const auto waveform_len  = GENERATE(1024, 2048, 4096);
  const auto num_waveforms = GENERATE(1, 2, 64, 256);
  const auto samplerate    = GENERATE(-16000, 0);

  auto waveforms      = std::vector<float>(waveform_len * num_waveforms);
  auto wavetable_data = adwt::WavetableData::build(
      waveforms, num_waveforms, static_cast<float>(samplerate));

  REQUIRE(wavetable_data.get() == nullptr);
}

TEST_CASE("Invalid waveform span") {
  const auto waveform_len  = GENERATE(1024, 2048, 4096);
  const auto num_waveforms = GENERATE(1, 2, 64, 256);
  const auto samplerate    = GENERATE(16000, 32000, 44100, 48000, 88200, 96000);
  const auto len_offset    = GENERATE(-2, -1, 1, 2);

  auto waveforms =
      std::vector<float>(waveform_len * num_waveforms + len_offset);
  auto wavetable_data = adwt::WavetableData::build(
      waveforms, num_waveforms, static_cast<float>(samplerate));

  INFO("waveforms : " << waveform_len * num_waveforms + len_offset);
  INFO("num_waveforms : " << num_waveforms);
  REQUIRE(wavetable_data.get() == nullptr);
}

TEST_CASE("Invalid num_waveforms") {
  const auto waveform_len  = GENERATE(1024, 2048, 4096);
  const auto num_waveforms = GENERATE(0, -1, -64);
  const auto samplerate    = GENERATE(16000, 32000, 44100, 48000, 88200, 96000);

  auto waveforms      = std::vector<float>(waveform_len * 64);
  auto wavetable_data = adwt::WavetableData::build(
      waveforms, num_waveforms, static_cast<float>(samplerate));

  REQUIRE(wavetable_data.get() == nullptr);
}

}  // namespace tests