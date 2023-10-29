/*
* juce_benchmark.cpp
* Created by maxmarsc, 08/10/2023
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
      auto phase_span =
          std::span(phase_sweep.begin() + i * upsampled_block_size,
                    phase_sweep.begin() + (i + 1) * upsampled_block_size);
      auto output_span = std::span(output_vec.begin() + i * block_size,
                                   output_vec.begin() + (i + 1) * block_size);
      osc.process(phase_span, output_span);
    }
  }
}

//NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweep)->RangeMultiplier(2)->Range(16, 2048);

}  // namespace benchmarks
