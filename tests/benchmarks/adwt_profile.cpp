/*
* adwt_benchmark.cpp
* Created by maxmarsc, 08/10/2023
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
  auto waveform_data = adwt::WaveformData::build(benchmarks::kSawWaveform, 1,
                                                 static_cast<int>(samplerate));
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WaveformData object" << std::endl;
    std::abort();
  }

  // Init the oscillator
  auto osc = adwt::Oscillator<adwt::FilterType::kType5>{};
  if (osc.init(std::move(waveform_data), std::make_tuple(0.99F, 0.01F)) != 0) {
    std::cerr << "Failed to init oscillator class" << std::endl;
    std::abort();
  }

  // Init the sweep phase data
  auto phase_sweep = std::vector<float>(num_frames);
  benchmarks::generateLinearSweepPhase(phase_sweep, 20.F, samplerate / 2.F,
                                       samplerate);

  // Prepare the processing block
  auto output_vec = std::vector<float>(num_frames);

  for (auto i : iter::range(1)) {
    auto phase_span = std::span(phase_sweep.begin() + i * block_size,
                                phase_sweep.begin() + (i + 1) * block_size);

    auto output_span = std::span(output_vec.begin() + i * block_size,
                                 output_vec.begin() + (i + 1) * block_size);
    osc.process<adwt::Direction::kForward>(phase_span, output_span);
  }
}

int main() {
  profileSweepUpOrder6(64);

  return 0;
}
