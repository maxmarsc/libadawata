/*
* downsampler_juce.cpp
* Created by maxmarsc, 14/10/2023
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

void DownsamplerJUCE::processDerived(std::span<float> src,
                                     std::span<float> dst) noexcept {
  std::memcpy(upsampled_block_.getChannelPointer(0), src.data(),
              src.size() * sizeof(float));
  auto* dst_data = dst.data();
  auto dst_block = juce::dsp::AudioBlock<float>(&dst_data, 1, dst.size());
  downsampler_.processSamplesDown(dst_block);
}

}  // namespace ovs