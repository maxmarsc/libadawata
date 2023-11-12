/*
* ovs_oscillator.hpp
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