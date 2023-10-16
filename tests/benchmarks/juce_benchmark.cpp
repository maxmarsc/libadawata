/*
* juce_benchmark.cpp
* Created by maxmarsc, 08/10/2023
*/

#include <benchmark/benchmark.h>
#include <cppitertools/range.hpp>
#include <iostream>
#include <sndfile.hh>

#include "source/ovs_oscillator.hpp"
#include "tests/benchmarks/utils_benchmarks.hpp"

namespace benchmarks {

void bmSweep(benchmark::State& state) {
  const auto block_size       = static_cast<int>(state.range(0));
  const int64_t num_frames    = 2048 * 100;
  const auto num_blocks       = num_frames / block_size;
  const auto samplerate       = 44100.F;
  const auto upsampling_ratio = 8;
  constexpr auto kSawWaveFile = std::string_view(ASSETS_DIR "/saw_2048.wav");

  // Load the waveform file
  auto waveform_sndfile   = SndfileHandle(kSawWaveFile.data(), SFM_READ);
  const auto waveform_len = waveform_sndfile.frames();
  auto waveform_vec =
      std::vector<float>(static_cast<std::size_t>(waveform_len));
  auto read = waveform_sndfile.readf(waveform_vec.data(), waveform_len);
  if (read != waveform_len || waveform_len != 2048) {
    std::cerr << "Failed to read waveform file" << std::endl;
    std::abort();
  }

  // Init the oscilator
  using Oscillator = ovs::OvsOscillator<ovs::ResamplingBackend::kJuce>;
  Oscillator osc(
      {ovs::DownsamplerJUCE::FilterType::filterHalfBandPolyphaseIIR, true});
  if (osc.init(waveform_vec, block_size, samplerate, upsampling_ratio) != 0) {
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

  // for (auto& val : output_vec) {
  //   val *= 0.5F;
  // }

  // if (SndfileHandle("juce_sweep_output.wav", SFM_WRITE,
  //                   waveform_sndfile.format(), 1, samplerate)
  //         .writef(output_vec.data(), num_frames) != num_frames)
  //   std::abort();
}

//NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
BENCHMARK(bmSweep)->RangeMultiplier(2)->Range(16, 2048);
// BENCHMARK(bmSweep)->Range(64, 64);

}  // namespace benchmarks
