/*
* downsampler_juce.hpp
* Created by maxmarsc, 14/10/2023
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

#pragma once

#include "downsampler_base.hpp"

#include <juce_core/juce_core.h>
#include <juce_dsp/juce_dsp.h>

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