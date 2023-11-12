/*
* juce_benchmark.cpp
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
#include <cppitertools/range.hpp>
#include <iostream>
#include <sndfile.hh>

#include "saw_2048.hpp"
#include "source/ovs_oscillator.hpp"
#include "tests/benchmarks/utils_benchmarks.hpp"

namespace benchmarks {

void bmSweep(benchmark::State& state) {
  const auto block_size       = static_cast<int>(state.range(0));
  const int64_t num_frames    = 2048 * 100;
  const auto num_blocks       = num_frames / block_size;
  const auto samplerate       = 44100.F;
  const auto upsampling_ratio = 8;

  // Init the oscilator
  using Oscillator = ovs::OvsOscillator<ovs::ResamplingBackend::kJuce>;
  Oscillator osc(
      {ovs::DownsamplerJUCE::FilterType::filterHalfBandPolyphaseIIR, true});
  if (osc.init(kSawWaveform, block_size, samplerate, upsampling_ratio) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
    std::abort();
  }

  // Init the sweep phase data
  auto phase_sweep =
      std::vector<float>(num_frames * static_cast<int64_t>(upsampling_ratio));
  generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F,
                           samplerate * upsampling_ratio);

  auto output_vec = std::vector<float>(num_frames);

  for (auto _ : state) {
    for (auto i : iter::range(num_blocks)) {
      const auto upsampled_block_size = upsampling_ratio * block_size;
      auto phase_span                 = adwt::Span<float>(
          phase_sweep.data() + i * upsampled_block_size, upsampled_block_size);
      auto output_span =
          adwt::Span<float>(output_vec.data() + i * block_size, block_size);
      osc.process(phase_span, output_span);
    }
  }
}

//NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweep)->RangeMultiplier(2)->Range(16, 2048);

}  // namespace benchmarks
