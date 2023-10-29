/*
* ovs_oscillator.cpp
* Created by maxmarsc, 08/10/2023
*/

#include <cppitertools/zip.hpp>
#include "ovs_oscillator.hpp"

#include "adwt/maths.hpp"

namespace ovs {

template <ResamplingBackend B>
OvsOscillator<B>::OvsOscillator(const DownsamplerCtorArgs<Downsampler<B>>& args)
    : downsampler_(args) {}

template <ResamplingBackend B>
[[nodiscard]] int OvsOscillator<B>::init(adwt::Span<const float> waveform,
                                         int block_size, float samplerate,
                                         int ratio) {
  if (ratio <= 0 || block_size <= 0)
    return 1;
  upsampled_data_.resize(static_cast<std::size_t>(block_size) *
                         static_cast<std::size_t>(ratio));
  ratio_ = ratio;
  waveform_.resize(waveform.size());
  std::copy(waveform.begin(), waveform.end(), waveform_.begin());

  if (downsampler_.init(block_size, samplerate, ratio) != 0)
    return 1;

  return 0;
}

template <ResamplingBackend B>
void OvsOscillator<B>::process(adwt::Span<float> phases,
                               adwt::Span<float> output) {
  assert(phases.size() == output.size() * static_cast<std::size_t>(ratio_));
  assert(phases.size() == upsampled_data_.size());

  const auto waveform_len = waveform_.size();

  // Naive interpolation
  for (auto&& [phase, sample] : iter::zip(phases, upsampled_data_)) {
    auto relative_idx = phase * waveform_len;
    const auto prev_idx =
        static_cast<std::size_t>(adwt::maths::floor(relative_idx));
    const auto next_idx = adwt::maths::reduce(prev_idx + 1, waveform_len);
    const auto a        = waveform_[next_idx] - waveform_[prev_idx];
    const auto b = waveform_[prev_idx] - static_cast<float>(prev_idx) * a;
    sample       = a * relative_idx + b;
  }

  downsampler_.process(upsampled_data_, output);
}

}  // namespace ovs

template class ovs::OvsOscillator<ovs::ResamplingBackend::kJuce>;