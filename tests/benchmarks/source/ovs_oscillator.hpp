/*
* ovs_oscillator.hpp
* Created by maxmarsc, 08/10/2023
*/

#pragma once

#include <vector>
#include "downsampler.hpp"

namespace ovs {

template <ResamplingBackend B>
class OvsOscillator {
 public:
  explicit OvsOscillator(const DownsamplerCtorArgs<Downsampler<B>>&);

  [[nodiscard]] int init(std::span<float> waveform, int block_size,
                         float samplerate, int ratio);

  void process(std::span<float> phases, std::span<float> output);
  void processRaw(std::span<float> phases, std::span<float> output);

 private:
  Downsampler<B> downsampler_;
  std::vector<float> upsampled_data_;
  std::vector<float> waveform_;
  int ratio_{};
};

}  // namespace ovs