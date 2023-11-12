/*
* oscillator_tests.cpp
* Created by maxmarsc, 28/09/2023
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

#include <catch2/catch_get_random_seed.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_all.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <iostream>
#include <optional>

#include <sndfile.hh>

#include "adwt/adwt.hpp"

namespace tests {

TEST_CASE("Constructor") {
  CHECK_NOTHROW(adwt::Oscillator<adwt::FilterType::kType1>{});
}

TEST_CASE("Invalid init") {
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>();

  REQUIRE(osc.init(nullptr) != 0);
}

TEST_CASE("Valid init BT2") {
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>();

  const auto waveform_len = GENERATE(1024, 2048);
  // const auto num_waveforms = GENERATE(1, 2, 16);
  const auto num_waveforms = 1;
  const auto samplerate    = GENERATE(44100, 48000);
  const auto waveforms     = std::vector<float>(waveform_len * num_waveforms);

  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);
  REQUIRE(waveform_data != nullptr);
  REQUIRE(osc.init(std::move(waveform_data)) == 0);
  CHECK(osc.crtWaveform() == 0);
  CHECK(osc.numWaveforms() == num_waveforms);

  auto rng_bi = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 1.F, Catch::getSeed());
  auto rng_fwd = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 0.5F, Catch::getSeed());

  SECTION("Then processing bidirectionnal") {
    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);

    for (auto& phase : phase_vector) {
      // This is only to check that we don't crash with complete random values
      // but the output of this will be complete trash
      rng_bi.next();
      phase = rng_bi.get();
    }

    auto count = 10;
    while (--count) {
      osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
                                                    output_vector);
    }
  }

  SECTION("Then processing forward") {
    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }

  SECTION("Then valid swap of same size and processing bidirectionnal") {
    auto waveform_data_same =
        adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

    REQUIRE(waveform_data_same != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(waveform_data_same)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);

    for (auto& phase : phase_vector) {
      // This is only to check that we don't crash with complete random values
      // but the output of this will be complete trash
      rng_bi.next();
      phase = rng_bi.get();
    }

    auto count = 10;
    while (--count) {
      osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
                                                    output_vector);
    }
  }

  SECTION("Then valid swap of same size and processing forward") {
    auto waveform_data_same =
        adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

    REQUIRE(waveform_data_same != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(waveform_data_same)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }

  SECTION("Then invalid swap") {
    auto ret = osc.swapWaveforms(nullptr);
    REQUIRE(ret == nullptr);
  }

  SECTION("Then valid swap of new size and processing bidirectionnal") {
    const auto new_waveform_len  = GENERATE(256, 512);
    const auto new_num_waveforms = GENERATE(1, 2, 4);
    const auto waveforms =
        std::vector<float>(new_waveform_len * new_num_waveforms);

    auto new_waveform_data =
        adwt::WaveformData::build(waveforms, new_num_waveforms, samplerate);

    REQUIRE(new_waveform_data != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(new_waveform_data)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);

    for (auto& phase : phase_vector) {
      // This is only to check that we don't crash with complete random values
      // but the output of this will be complete trash
      rng_bi.next();
      phase = rng_bi.get();
    }

    auto count = 10;
    while (--count) {
      osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
                                                    output_vector);
    }
  }

  SECTION("Then valid swap of new size and processing forward") {
    const auto new_waveform_len  = GENERATE(256, 512);
    const auto new_num_waveforms = GENERATE(1, 2, 4);
    const auto waveforms =
        std::vector<float>(new_waveform_len * new_num_waveforms);

    auto new_waveform_data =
        adwt::WaveformData::build(waveforms, new_num_waveforms, samplerate);

    REQUIRE(new_waveform_data != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(new_waveform_data)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }
}

TEST_CASE("Valid init O6") {
  auto osc = adwt::Oscillator<adwt::FilterType::kType4>();

  const auto waveform_len = GENERATE(2048);
  // const auto num_waveforms = GENERATE(1, 2, 16);
  const auto num_waveforms = 1;
  const auto samplerate    = GENERATE(44100);
  const auto waveforms     = std::vector<float>(waveform_len * num_waveforms);

  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);
  REQUIRE(waveform_data != nullptr);
  REQUIRE(osc.init(std::move(waveform_data)) == 0);
  CHECK(osc.crtWaveform() == 0);
  CHECK(osc.numWaveforms() == num_waveforms);

  auto rng_bi = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 1.F, Catch::getSeed());
  auto rng_fwd = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 0.5F, Catch::getSeed());

  // SECTION("Then processing bidirectionnal") {
  //   const auto block_size = GENERATE(16, 32, 64, 128);
  //   auto phase_vector     = std::vector<float>(block_size);
  //   auto output_vector    = std::vector<float>(block_size);

  //   for (auto& phase : phase_vector) {
  //     // This is only to check that we don't crash with complete random values
  //     // but the output of this will be complete trash
  //     rng_bi.next();
  //     phase = rng_bi.get();
  //   }

  //   auto count = 10;
  //   while (--count) {
  //     osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
  //                                                   output_vector);
  //   }
  // }

  SECTION("Then processing forward") {
    const auto block_size = GENERATE(128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }
}

TEST_CASE("Valid init O8") {
  auto osc = adwt::Oscillator<adwt::FilterType::kType5>();

  const auto waveform_len = GENERATE(2048);
  // const auto num_waveforms = GENERATE(1, 2, 16);
  const auto num_waveforms = 1;
  const auto samplerate    = GENERATE(44100);
  const auto waveforms     = std::vector<float>(waveform_len * num_waveforms);

  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);
  REQUIRE(waveform_data != nullptr);
  REQUIRE(osc.init(std::move(waveform_data)) == 0);
  CHECK(osc.crtWaveform() == 0);
  CHECK(osc.numWaveforms() == num_waveforms);

  auto rng_bi = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 1.F, Catch::getSeed());
  auto rng_fwd = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 0.5F, Catch::getSeed());

  // SECTION("Then processing bidirectionnal") {
  //   const auto block_size = GENERATE(16, 32, 64, 128);
  //   auto phase_vector     = std::vector<float>(block_size);
  //   auto output_vector    = std::vector<float>(block_size);

  //   for (auto& phase : phase_vector) {
  //     // This is only to check that we don't crash with complete random values
  //     // but the output of this will be complete trash
  //     rng_bi.next();
  //     phase = rng_bi.get();
  //   }

  //   auto count = 10;
  //   while (--count) {
  //     osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
  //                                                   output_vector);
  //   }
  // }

  SECTION("Then processing forward") {
    const auto block_size = GENERATE(128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }
}

TEST_CASE("Valid init CH10") {
  auto osc = adwt::Oscillator<adwt::FilterType::kType2>();

  const auto waveform_len = GENERATE(1024, 2048);
  // const auto num_waveforms = GENERATE(1, 2, 16);
  const auto num_waveforms = 1;
  const auto samplerate    = GENERATE(44100, 48000);
  const auto waveforms     = std::vector<float>(waveform_len * num_waveforms);

  auto waveform_data =
      adwt::WaveformData::build(waveforms, num_waveforms, samplerate);
  REQUIRE(waveform_data != nullptr);
  REQUIRE(osc.init(std::move(waveform_data)) == 0);
  CHECK(osc.crtWaveform() == 0);
  CHECK(osc.numWaveforms() == num_waveforms);

  auto rng_bi = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 1.F, Catch::getSeed());
  auto rng_fwd = Catch::Generators::RandomFloatingGenerator<float>(
      0.F, 0.5F, Catch::getSeed());

  SECTION("Then processing bidirectionnal") {
    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);

    for (auto& phase : phase_vector) {
      // This is only to check that we don't crash with complete random values
      // but the output of this will be complete trash
      rng_bi.next();
      phase = rng_bi.get();
    }

    auto count = 10;
    while (--count) {
      osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
                                                    output_vector);
    }
  }

  SECTION("Then processing forward") {
    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }

  SECTION("Then valid swap of same size and processing bidirectionnal") {
    auto waveform_data_same =
        adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

    REQUIRE(waveform_data_same != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(waveform_data_same)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);

    for (auto& phase : phase_vector) {
      // This is only to check that we don't crash with complete random values
      // but the output of this will be complete trash
      rng_bi.next();
      phase = rng_bi.get();
    }

    auto count = 10;
    while (--count) {
      osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
                                                    output_vector);
    }
  }

  SECTION("Then valid swap of same size and processing forward") {
    auto waveform_data_same =
        adwt::WaveformData::build(waveforms, num_waveforms, samplerate);

    REQUIRE(waveform_data_same != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(waveform_data_same)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }

  SECTION("Then invalid swap") {
    auto ret = osc.swapWaveforms(nullptr);
    REQUIRE(ret == nullptr);
  }

  SECTION("Then valid swap of new size and processing bidirectionnal") {
    const auto new_waveform_len  = GENERATE(256, 512);
    const auto new_num_waveforms = GENERATE(1, 2, 4);
    const auto waveforms =
        std::vector<float>(new_waveform_len * new_num_waveforms);

    auto new_waveform_data =
        adwt::WaveformData::build(waveforms, new_num_waveforms, samplerate);

    REQUIRE(new_waveform_data != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(new_waveform_data)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);

    for (auto& phase : phase_vector) {
      // This is only to check that we don't crash with complete random values
      // but the output of this will be complete trash
      rng_bi.next();
      phase = rng_bi.get();
    }

    auto count = 10;
    while (--count) {
      osc.process<adwt::Direction::kBidirectionnal>(phase_vector,
                                                    output_vector);
    }
  }

  SECTION("Then valid swap of new size and processing forward") {
    const auto new_waveform_len  = GENERATE(256, 512);
    const auto new_num_waveforms = GENERATE(1, 2, 4);
    const auto waveforms =
        std::vector<float>(new_waveform_len * new_num_waveforms);

    auto new_waveform_data =
        adwt::WaveformData::build(waveforms, new_num_waveforms, samplerate);

    REQUIRE(new_waveform_data != nullptr);
    REQUIRE(osc.swapWaveforms(std::move(new_waveform_data)) != nullptr);

    const auto block_size = GENERATE(16, 32, 64, 128);
    auto phase_vector     = std::vector<float>(block_size);
    auto output_vector    = std::vector<float>(block_size);
    auto prev_phase       = 0.F;

    auto count = 10;
    while (--count) {
      for (auto& phase : phase_vector) {
        // This is only to check that we don't crash with complete random values
        // but the output of this will be complete trash
        rng_fwd.next();
        phase      = adwt::maths::reduce(prev_phase + rng_fwd.get(), 1.F);
        prev_phase = phase;
      }
      osc.process<adwt::Direction::kForward>(phase_vector, output_vector);
    }
  }
}

TEST_CASE("Reference test : BT2 sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/BT2_w2048_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 2.6e-5F;  // -43dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain = 0.5F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>{};
  REQUIRE(osc.init(std::move(waveform_data)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  SECTION("Process bi as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    // auto output_file = SndfileHandle("BT2_sweep_test.wav", SFM_WRITE,
    //                                  output_sndfile.format(), 1, 44100);
    // REQUIRE(output_file.writef(output_vec.data(), output_vec.size()) ==
    //         output_vec.size());

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }

  SECTION("Process fwd as single block") {
    osc.process<adwt::Direction::kForward>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : CH10 sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/CH10_w2048_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 2.6e-4F;  // -33dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain          = 0.5F;
  constexpr auto kPhaseInit     = 0.F;
  constexpr auto kPhaseDiffInit = 0.00045351473922902497F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType2>{};
  REQUIRE(osc.init(std::move(waveform_data),
                   std::make_tuple(kPhaseInit, kPhaseDiffInit)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }

  SECTION("Process fwd as single block") {
    osc.process<adwt::Direction::kForward>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : BT2 flipped sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/flipped_sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/BT2_w2048_flipped_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 5e-5F;  // -40dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain          = 0.5F;
  constexpr auto kPhaseInit     = 0.7497732426418224F;
  constexpr auto kPhaseDiffInit = -0.5F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>{};
  REQUIRE(osc.init(std::move(waveform_data),
                   std::make_tuple(kPhaseInit, kPhaseDiffInit)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == kPhaseInit);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  const auto size  = 2048;
  auto output_span = adwt::Span<float>(output_vec.data(), size);
  auto ref_span    = adwt::Span<float>(output_ref_vec.data(), size);

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    auto imax    = 0;
    auto i       = 0;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      if (std::abs(val - ref) > err_max) {
        imax = i;
      }
      err_max = std::max(err_max, std::abs(val - ref));
      ++i;
    }

    INFO("imax " << imax);
    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : CH10 flipped sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/flipped_sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/CH10_w2048_flipped_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 2e-4F;  // -33dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain          = 0.5F;
  constexpr auto kPhaseInit     = 0.7497732426418224F;
  constexpr auto kPhaseDiffInit = -0.5F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType2>{};
  REQUIRE(osc.init(std::move(waveform_data),
                   std::make_tuple(kPhaseInit, kPhaseDiffInit)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == kPhaseInit);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  const auto size  = 2048;
  auto output_span = adwt::Span<float>(output_vec.data(), size);
  auto ref_span    = adwt::Span<float>(output_ref_vec.data(), size);

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    auto imax    = 0;
    auto i       = 0;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      if (std::abs(val - ref) > err_max) {
        imax = i;
      }
      err_max = std::max(err_max, std::abs(val - ref));
      ++i;
    }

    INFO("imax " << imax);
    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : BT2 reverse sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/reverse_sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/BT2_w2048_reverse_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 9e-5F;  // -37dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain          = 0.5F;
  constexpr auto kPhaseInit     = 0.F;
  constexpr auto kPhaseDiffInit = 0.5F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>{};
  REQUIRE(osc.init(std::move(waveform_data),
                   std::make_tuple(kPhaseInit, kPhaseDiffInit)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == kPhaseInit);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }

  SECTION("Process fwd as single block") {
    osc.process<adwt::Direction::kForward>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : CH10 reverse sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/reverse_sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/CH10_w2048_reverse_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 6e-4F;  // -30dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain          = 0.5F;
  constexpr auto kPhaseInit     = 0.F;
  constexpr auto kPhaseDiffInit = 0.5F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType2>{};
  REQUIRE(osc.init(std::move(waveform_data),
                   std::make_tuple(kPhaseInit, kPhaseDiffInit)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == kPhaseInit);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }

  SECTION("Process fwd as single block") {
    osc.process<adwt::Direction::kForward>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : BT2 reverse flipped sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/reverse_flipped_sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/BT2_w2048_reverse_flipped_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 5e-5F;  // -30dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain          = 0.5F;
  constexpr auto kPhaseInit     = 0.7497732426272705F;
  constexpr auto kPhaseDiffInit = -0.0004535147381830029F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>{};
  REQUIRE(osc.init(std::move(waveform_data),
                   std::make_tuple(kPhaseInit, kPhaseDiffInit)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == kPhaseInit);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  const auto size  = 2048;
  auto output_span = adwt::Span<float>(output_vec.data(), size);
  auto ref_span    = adwt::Span<float>(output_ref_vec.data(), size);

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    auto imax    = 0;
    auto i       = 0;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      if (std::abs(val - ref) > err_max) {
        imax = i;
      }
      err_max = std::max(err_max, std::abs(val - ref));
      ++i;
    }

    INFO("imax " << imax);
    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : CH10 reverse flipped sweep") {
  constexpr auto kSweepPhaseFile =
      std::string_view(ASSETS_DIR "/reverse_flipped_sweep_phase.wav");
  constexpr auto kSweepAudioFile =
      std::string_view(ASSETS_DIR "/CH10_w2048_reverse_flipped_sweep.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric
  constexpr auto kEps = 2e-4F;  // -33dB difference with python impl
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain          = 0.5F;
  constexpr auto kPhaseInit     = 0.7497732426272705F;
  constexpr auto kPhaseDiffInit = -0.0004535147381830029F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType2>{};
  REQUIRE(osc.init(std::move(waveform_data),
                   std::make_tuple(kPhaseInit, kPhaseDiffInit)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(kSweepPhaseFile.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == kPhaseInit);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(kSweepAudioFile.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  const auto size  = 2048;
  auto output_span = adwt::Span<float>(output_vec.data(), size);
  auto ref_span    = adwt::Span<float>(output_ref_vec.data(), size);

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    // SndfileHandle("CH10_reverse_flipped_sweep.wav", SFM_WRITE,
    //               output_sndfile.format(), 1, 44100)
    //     .writef(output_vec.data(), output_sndfile.frames());

    auto err_max = 0.F;
    auto imax    = 0;
    auto i       = 0;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      if (std::abs(val - ref) > err_max) {
        imax = i;
      }
      err_max = std::max(err_max, std::abs(val - ref));
      ++i;
    }

    INFO("imax " << imax);
    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : BT2 460Hz") {
  constexpr auto k460_Phase_File =
      std::string_view(ASSETS_DIR "/460Hz_phase.wav");
  constexpr auto k460_Audio_File =
      std::string_view(ASSETS_DIR "/BT2_w2048_460Hz.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric, checked with matlab SNR's, it gives the same value of 33.18dB
  // -43dB difference with python impl
  constexpr auto kEps                  = 2e-5F;
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain = 0.5F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>{};
  REQUIRE(osc.init(std::move(waveform_data)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(k460_Phase_File.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(k460_Audio_File.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    // Apply gain reduction to match output ref
    for (auto& output_val : output_vec) {
      output_val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }
    CHECK(err_max <= kEps);
  }

  SECTION("Process fwd as single block") {
    osc.process<adwt::Direction::kForward>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }
}

TEST_CASE("Reference test : CH10 460Hz") {
  constexpr auto k460_Phase_File =
      std::string_view(ASSETS_DIR "/460Hz_phase.wav");
  constexpr auto k460_Audio_File =
      std::string_view(ASSETS_DIR "/CH10_w2048_460Hz.wav");
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Empiric, checked with matlab SNR's, it gives the same value of 33.18dB
  // -43dB difference with python impl
  constexpr auto kEps                  = 5e-5F;
  Catch::StringMaker<float>::precision = 15;
  // The reference file has a gain of 0.5
  constexpr auto kGain = 0.5F;

  // Load the waveform file
  auto waveform_sndfile = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  REQUIRE(waveform_sndfile.frames() != 0);
  REQUIRE(waveform_sndfile.channels() == 1);
  const auto samplerate   = waveform_sndfile.samplerate();
  const auto waveform_len = waveform_sndfile.frames();

  // Read the waveform file
  auto waveform_vector = std::vector<float>(waveform_len);
  REQUIRE(waveform_sndfile.readf(waveform_vector.data(), waveform_len) ==
          waveform_len);

  // Init the waveform data
  auto waveform_data =
      adwt::WaveformData::build(waveform_vector, 1, samplerate);
  REQUIRE(waveform_data != nullptr);

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType2>{};
  REQUIRE(osc.init(std::move(waveform_data)) == 0);

  // Init & read the phase file
  auto phase_sndfile = SndfileHandle(k460_Phase_File.data(), SFM_READ);
  float first_sample{};
  REQUIRE(phase_sndfile.channels() == 1);
  const auto num_samples = phase_sndfile.frames() - 1;
  // We throw the first sample away, it's zero
  REQUIRE(phase_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto phase_vec = std::vector<float>(num_samples);
  REQUIRE(phase_sndfile.readf(phase_vec.data(), num_samples) == num_samples);

  // Init and read the output ref file
  auto output_sndfile = SndfileHandle(k460_Audio_File.data(), SFM_READ);
  REQUIRE(output_sndfile.channels() == 1);
  REQUIRE(output_sndfile.frames() == phase_sndfile.frames());
  // We throw the first sample away, it's zero
  REQUIRE(output_sndfile.readf(&first_sample, 1) == 1);
  REQUIRE(first_sample == 0.F);
  auto output_ref_vec = std::vector<float>(num_samples);
  REQUIRE(output_sndfile.readf(output_ref_vec.data(), num_samples) ==
          num_samples);

  // Create vector for the output
  auto output_vec = std::vector<float>(num_samples);
  for (auto& phase : phase_vec) {
    phase = std::fmod(phase, 1.F);
  }

  const auto size  = 512;
  const auto start = 209090 - 32;
  auto input_span  = adwt::Span<float>(phase_vec.data() + start, size);
  auto output_span = adwt::Span<float>(output_vec.data() + start, size);
  auto ref_span    = adwt::Span<float>(output_ref_vec.data() + start, size);
  auto err_vec     = std::vector<float>(num_samples);

  SECTION("Process as single block") {
    osc.process<adwt::Direction::kBidirectionnal>(phase_vec, output_vec);

    // Apply gain reduction to match output ref
    for (auto& output_val : output_vec) {
      output_val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }
    CHECK(err_max <= kEps);
  }

  SECTION("Process fwd as single block") {
    osc.process<adwt::Direction::kForward>(phase_vec, output_vec);

    for (auto& val : output_vec) {
      val *= kGain;
    }

    auto err_max = 0.F;
    for (auto&& [val, ref] : iter::zip(output_vec, output_ref_vec)) {
      err_max = std::max(err_max, std::abs(val - ref));
    }

    CHECK(err_max <= kEps);
  }
}

}  // namespace tests