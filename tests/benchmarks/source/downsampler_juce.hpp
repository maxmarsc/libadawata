/*
* downsampler_juce.hpp
* Created by maxmarsc, 14/10/2023
*/

#pragma once

#include "downsampler_base.hpp"

#include <juce_core/juce_core.h>
#include <juce_dsp/juce_dsp.h>
// #include "juce_dsp/processors/juce_Oversampling.h"

namespace ovs {

class DownsamplerJUCE;

template <>
struct DownsamplerCtorArgs<DownsamplerJUCE> {
  juce::dsp::Oversampling<float>::FilterType ftype;
  bool max_quality;
};

class DownsamplerJUCE final : public DownsamplerBase<DownsamplerJUCE> {
  friend class DownsamplerBase<DownsamplerJUCE>;

 public:
  using FilterType = juce::dsp::Oversampling<float>::FilterType;
  using CtorArgs   = DownsamplerCtorArgs<DownsamplerJUCE>;

  explicit DownsamplerJUCE(const CtorArgs& args);

 protected:
  [[nodiscard]] int initDerived(int block_size, float samplerate, int ratio);
  void processDerived(adwt::Span<float> src, adwt::Span<float> dst) noexcept;

 private:
  juce::dsp::Oversampling<float> downsampler_;
  juce::dsp::AudioBlock<float> upsampled_block_;
};

}  // namespace ovs