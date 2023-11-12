/*
* downsampler_juce.cpp
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

#include "downsampler_juce.hpp"

namespace ovs {

DownsamplerJUCE::DownsamplerJUCE(const DownsamplerJUCE::CtorArgs& args)
    : downsampler_(1, 3, args.ftype, args.max_quality) {}

[[nodiscard]] int DownsamplerJUCE::initDerived(int block_size,
                                               float /*samplerate*/,
                                               int /*ratio*/) {
  downsampler_.initProcessing(static_cast<std::size_t>(block_size));
  auto ds_vec       = std::vector<float>(static_cast<std::size_t>(block_size));
  auto* ds_vec_data = ds_vec.data();
  auto ds_block     = juce::dsp::AudioBlock<float>(
      &ds_vec_data, 1, static_cast<std::size_t>(block_size));
  upsampled_block_ = downsampler_.processSamplesUp(ds_block);
  return 0;
}

void DownsamplerJUCE::processDerived(adwt::Span<float> src,
                                     adwt::Span<float> dst) noexcept {
  std::memcpy(upsampled_block_.getChannelPointer(0), src.data(),
              src.size() * sizeof(float));
  auto* dst_data = dst.data();
  auto dst_block = juce::dsp::AudioBlock<float>(&dst_data, 1, dst.size());
  downsampler_.processSamplesDown(dst_block);
}

}  // namespace ovs