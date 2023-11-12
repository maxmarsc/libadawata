/*
* adwt_benchmark.cpp
* Created by maxmarsc, 08/10/2023
*
* libadawata benchmarks
* Copyright (C) 2023  Maxime Coutant
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <benchmark/benchmark.h>
#include <iostream>
#include <sndfile.hh>

#include "adwt/adwt.hpp"
#include "saw_2048.hpp"
#include "tests/benchmarks/utils_benchmarks.hpp"

namespace benchmarks {

void bmSweepUpOrder2(benchmark::State& state) {
  const auto block_size = state.range(0);
  const auto num_frames = 2048 * 100;
  const auto num_blocks = num_frames / block_size;
  const auto samplerate = 44100.F;

  // Init the waveform data
  auto waveform_data =
      adwt::WavetableData::build(kSawWaveform, 1, static_cast<int>(samplerate));
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    std::abort();
  }

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType1>{};
  if (osc.init(std::move(waveform_data), std::make_tuple(0.99F, 0.01F)) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
    std::abort();
  }

  // Init the sweep phase data
  auto phase_sweep = std::vector<float>(num_frames);
  generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F, samplerate);

  // Prepare the processing block
  auto output_vec = std::vector<float>(num_frames);

  for (auto _ : state) {
    for (auto i : iter::range(num_blocks)) {
      auto phase_span =
          adwt::Span<float>(phase_sweep.data() + i * block_size, block_size);

      auto output_span =
          adwt::Span<float>(output_vec.data() + i * block_size, block_size);
      osc.process<adwt::Direction::kForward>(phase_span, output_span);
    }
  }
}

void bmSweepUpOrder4(benchmark::State& state) {
  const auto block_size = state.range(0);
  const auto num_frames = 2048 * 100;
  const auto num_blocks = num_frames / block_size;
  const auto samplerate = 44100.F;

  // Init the waveform data
  auto waveform_data =
      adwt::WavetableData::build(kSawWaveform, 1, static_cast<int>(samplerate));
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    std::abort();
  }

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType3>{};
  if (osc.init(std::move(waveform_data)) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
  }

  // Init the sweep phase data
  auto phase_sweep = std::vector<float>(num_frames);
  generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F, samplerate);

  // Prepare the processing block
  auto block_dst = std::vector<float>(block_size);

  for (auto _ : state) {
    for (auto i : iter::range(num_blocks)) {
      auto phase_span =
          adwt::Span<float>(phase_sweep.data() + i * block_size, block_size);
      osc.process<adwt::Direction::kForward>(phase_span, block_dst);
    }
  }
}

void bmSweepUpOrder6(benchmark::State& state) {
  const auto block_size = state.range(0);
  const auto num_frames = 2048 * 100;
  const auto num_blocks = num_frames / block_size;
  const auto samplerate = 44100.F;

  // Init the waveform data
  auto waveform_data =
      adwt::WavetableData::build(kSawWaveform, 1, static_cast<int>(samplerate));
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    std::abort();
  }

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType4>{};
  if (osc.init(std::move(waveform_data)) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
  }

  // Init the sweep phase data
  auto phase_sweep = std::vector<float>(num_frames);
  generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F, samplerate);

  // Prepare the processing block
  auto block_dst = std::vector<float>(block_size);

  for (auto _ : state) {
    for (auto i : iter::range(num_blocks)) {
      auto phase_span =
          adwt::Span<float>(phase_sweep.data() + i * block_size, block_size);
      osc.process<adwt::Direction::kForward>(phase_span, block_dst);
    }
  }
}

void bmSweepUpOrder8(benchmark::State& state) {
  const auto block_size = state.range(0);
  const auto num_frames = 2048 * 100;
  const auto num_blocks = num_frames / block_size;
  const auto samplerate = 44100.F;

  // Init the waveform data
  auto waveform_data =
      adwt::WavetableData::build(kSawWaveform, 1, static_cast<int>(samplerate));
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    std::abort();
  }

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType5>{};
  if (osc.init(std::move(waveform_data)) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
  }

  // Init the sweep phase data
  auto phase_sweep = std::vector<float>(num_frames);
  generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F, samplerate);

  // Prepare the processing block
  auto block_dst = std::vector<float>(block_size);

  for (auto _ : state) {
    for (auto i : iter::range(num_blocks)) {
      auto phase_span =
          adwt::Span<float>(phase_sweep.data() + i * block_size, block_size);
      osc.process<adwt::Direction::kForward>(phase_span, block_dst);
    }
  }
}

void bmSweepUpOrder10(benchmark::State& state) {
  const auto block_size = state.range(0);
  const auto num_frames = 2048 * 100;
  const auto num_blocks = num_frames / block_size;
  const auto samplerate = 44100.F;

  // Init the waveform data
  auto waveform_data =
      adwt::WavetableData::build(kSawWaveform, 1, static_cast<int>(samplerate));
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    std::abort();
  }

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType2>{};
  if (osc.init(std::move(waveform_data)) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
  }

  // Init the sweep phase data
  auto phase_sweep = std::vector<float>(num_frames);
  generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F, samplerate);

  // Prepare the processing block
  auto block_dst = std::vector<float>(block_size);

  for (auto _ : state) {
    for (auto i : iter::range(num_blocks)) {
      auto phase_span =
          adwt::Span<float>(phase_sweep.data() + i * block_size, block_size);
      osc.process<adwt::Direction::kForward>(phase_span, block_dst);
    }
  }
}

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweepUpOrder2)->RangeMultiplier(2)->Range(16, 2048);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweepUpOrder4)->RangeMultiplier(2)->Range(16, 2048);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweepUpOrder6)->RangeMultiplier(2)->Range(16, 2048);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweepUpOrder8)->RangeMultiplier(2)->Range(16, 2048);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweepUpOrder10)->RangeMultiplier(2)->Range(16, 2048);

}  // namespace benchmarks
