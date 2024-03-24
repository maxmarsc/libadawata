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

  auto wavetable_data = adwt::WavetableData::build(
      waveforms, num_waveforms, static_cast<float>(samplerate));
  REQUIRE(wavetable_data != nullptr);
  REQUIRE(osc.init(std::move(wavetable_data)) == 0);
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
    auto wavetable_data_same = adwt::WavetableData::build(
        waveforms, num_waveforms, static_cast<float>(samplerate));

    REQUIRE(wavetable_data_same != nullptr);
    REQUIRE(osc.swapWavetable(std::move(wavetable_data_same)) != nullptr);

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
    auto wavetable_data_same = adwt::WavetableData::build(
        waveforms, num_waveforms, static_cast<float>(samplerate));

    REQUIRE(wavetable_data_same != nullptr);
    REQUIRE(osc.swapWavetable(std::move(wavetable_data_same)) != nullptr);

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
    auto ret = osc.swapWavetable(nullptr);
    REQUIRE(ret == nullptr);
  }

  SECTION("Then valid swap of new size and processing bidirectionnal") {
    const auto new_waveform_len  = GENERATE(256, 512);
    const auto new_num_waveforms = GENERATE(1, 2, 4);
    const auto waveforms =
        std::vector<float>(new_waveform_len * new_num_waveforms);

    auto new_wavetable_data = adwt::WavetableData::build(
        waveforms, new_num_waveforms, static_cast<float>(samplerate));

    REQUIRE(new_wavetable_data != nullptr);
    REQUIRE(osc.swapWavetable(std::move(new_wavetable_data)) != nullptr);

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

    auto new_wavetable_data = adwt::WavetableData::build(
        waveforms, new_num_waveforms, static_cast<float>(samplerate));

    REQUIRE(new_wavetable_data != nullptr);
    REQUIRE(osc.swapWavetable(std::move(new_wavetable_data)) != nullptr);

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

  auto wavetable_data = adwt::WavetableData::build(
      waveforms, num_waveforms, static_cast<float>(samplerate));
  REQUIRE(wavetable_data != nullptr);
  REQUIRE(osc.init(std::move(wavetable_data)) == 0);
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

  auto wavetable_data = adwt::WavetableData::build(
      waveforms, num_waveforms, static_cast<float>(samplerate));
  REQUIRE(wavetable_data != nullptr);
  REQUIRE(osc.init(std::move(wavetable_data)) == 0);
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

  auto wavetable_data = adwt::WavetableData::build(
      waveforms, num_waveforms, static_cast<float>(samplerate));
  REQUIRE(wavetable_data != nullptr);
  REQUIRE(osc.init(std::move(wavetable_data)) == 0);
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
    auto wavetable_data_same = adwt::WavetableData::build(
        waveforms, num_waveforms, static_cast<float>(samplerate));

    REQUIRE(wavetable_data_same != nullptr);
    REQUIRE(osc.swapWavetable(std::move(wavetable_data_same)) != nullptr);

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
    auto wavetable_data_same = adwt::WavetableData::build(
        waveforms, num_waveforms, static_cast<float>(samplerate));

    REQUIRE(wavetable_data_same != nullptr);
    REQUIRE(osc.swapWavetable(std::move(wavetable_data_same)) != nullptr);

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
    auto ret = osc.swapWavetable(nullptr);
    REQUIRE(ret == nullptr);
  }

  SECTION("Then valid swap of new size and processing bidirectionnal") {
    const auto new_waveform_len  = GENERATE(256, 512);
    const auto new_num_waveforms = GENERATE(1, 2, 4);
    const auto waveforms =
        std::vector<float>(new_waveform_len * new_num_waveforms);

    auto new_wavetable_data = adwt::WavetableData::build(
        waveforms, new_num_waveforms, static_cast<float>(samplerate));

    REQUIRE(new_wavetable_data != nullptr);
    REQUIRE(osc.swapWavetable(std::move(new_wavetable_data)) != nullptr);

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

    auto new_wavetable_data = adwt::WavetableData::build(
        waveforms, new_num_waveforms, static_cast<float>(samplerate));

    REQUIRE(new_wavetable_data != nullptr);
    REQUIRE(osc.swapWavetable(std::move(new_wavetable_data)) != nullptr);

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

}  // namespace tests