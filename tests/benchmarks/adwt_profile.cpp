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

#include <iostream>
#include <sndfile.hh>

#include "adwt/adwt.hpp"
#include "saw_2048.hpp"
#include "tests/benchmarks/utils_benchmarks.hpp"

void profileSweepUpOrder6(int block_size) {
  // const auto block_size = state.range(0);
  const auto num_frames = 2048 * 100;
  const auto num_blocks = num_frames / block_size;
  const auto samplerate = 44100.F;

  // Init the waveform data
  auto wavetable_data = adwt::WavetableData::build(
      benchmarks::kSawWaveform, 1, static_cast<int>(samplerate));
  if (wavetable_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    std::abort();
  }

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType5>{};
  if (osc.init(std::move(wavetable_data), std::make_tuple(0.99F, 0.01F)) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
    std::abort();
  }

  // Init the sweep phase data
  auto phase_sweep = std::vector<float>(num_frames);
  benchmarks::generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F,
                                       samplerate);

  // Prepare the processing block
  auto output_vec = std::vector<float>(num_frames);

  for (auto i : iter::range(num_blocks)) {
    auto phase_span =
        adwt::Span<float>(phase_sweep.data() + i * block_size, block_size);

    auto output_span =
        adwt::Span<float>(output_vec.data() + i * block_size, block_size);
    osc.process<adwt::Direction::kForward>(phase_span, output_span);
  }
}

int main() {
  profileSweepUpOrder6(64);

  return 0;
}
