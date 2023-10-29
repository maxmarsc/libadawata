/*
* ovs_oscillator.hpp
* Created by maxmarsc, 08/10/2023
*/

#pragma once

#include <vector>
#include "adwt/span.hpp"
#include "downsampler.hpp"

namespace ovs {

template <ResamplingBackend B>
class OvsOscillator {
 public:
  explicit OvsOscillator(const DownsamplerCtorArgs<Downsampler<B>>&);

  [[nodiscard]] int init(adwt::Span<const float> waveform, int block_size,
                         float samplerate, int ratio);

  void process(adwt::Span<float> phases, adwt::Span<float> output);
  void processRaw(adwt::Span<float> phases, adwt::Span<float> output);

 private:
  Downsampler<B> downsampler_;
  std::vector<float> upsampled_data_;
  std::vector<float> waveform_;
  int ratio_{};
};

}  // namespace ovs