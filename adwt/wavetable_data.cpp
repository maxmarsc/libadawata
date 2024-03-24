/*
* wavetable_data.cpp
* Created by maxmarsc, 25/09/2023
*
* This work is licensed under the MIT License
*
* Copyright (c) 2023 Maxime Coutant
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#include "wavetable_data.hpp"

#include <cmath>

#include <samplerate.h>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <iostream>
#include <numeric>

#include "maths.hpp"

namespace adwt {

inline float computeM(float x0, float x1, float y0, float y1) {
  return (y1 - y0) / (x1 - x0);
}

inline float computeQ(float x0, float x1, float y0, float y1) {
  return (y0 * (x1 - x0) - x0 * (y1 - y0)) / (x1 - x0);
}

//==============================================================================
WavetableData::WavetableData(Span<const float> waveforms, int num_waveforms,
                             float samplerate, float mipmap_ratio)
    : mipmap_ratio_((1.F + mipmap_ratio) / 2.F) {
  // Resize the vectors to be sure then can hold enough waveforms
  m_.resize(num_waveforms);
  q_.resize(num_waveforms);
  m_diff_.resize(num_waveforms);
  q_diff_.resize(num_waveforms);

  const auto og_waveform_len =
      static_cast<int>(waveforms.size()) / num_waveforms;

  // Compute the mipmap scale and how many entries in the mipmap tables
  computeMipMapScale(og_waveform_len, samplerate);
  assert(numMipMapTables() > 0);

  // Compute the phase points vectors
  phases_.resize(numMipMapTables());
  for (auto mipmap_idx : iter::range(numMipMapTables())) {
    const auto waveform_len = og_waveform_len / (1 << mipmap_idx);
    computePhaseVector(waveform_len, mipmap_idx);
  }

  // We iterate over each waveform
  for (auto waveform_idx : iter::range(num_waveforms)) {
    auto og_waveform =
        waveforms.subspan(waveform_idx * og_waveform_len, og_waveform_len);

    // Compute MQ for the og waveform
    m_[waveform_idx].resize(numMipMapTables());
    q_[waveform_idx].resize(numMipMapTables());
    m_diff_[waveform_idx].resize(numMipMapTables());
    q_diff_[waveform_idx].resize(numMipMapTables());
    computeMQValues(og_waveform, waveform_idx, 0);

    // libsamplerate supports a maximum resampling ratio of 256
    constexpr auto kMaxLibSamplerate = 8;
    const auto num_intermediate_tables =
        std::max(numMipMapTables() - kMaxLibSamplerate - 1, 0);
    auto intermediate_tables =
        std::vector<std::vector<float>>(num_intermediate_tables);

    // Compute MQ for each mipmap entry
    for (const auto mipmap_idx : iter::range(1, numMipMapTables())) {
      if (mipmap_idx <= kMaxLibSamplerate) {
        auto ratio            = 2 << (mipmap_idx - 1);
        auto downsampling_src = og_waveform;

        const auto ds_waveform = downsampleWaveform(og_waveform, ratio);
        if (ds_waveform.empty())
          throw std::runtime_error("Downsampled failed");

        if (mipmap_idx <= num_intermediate_tables)
          intermediate_tables[mipmap_idx - 1] = std::move(ds_waveform);

        computeMQValues(ds_waveform, waveform_idx, mipmap_idx);
      } else {
        auto ratio            = 256;
        auto downsampling_src = Span<const float>(
            intermediate_tables[mipmap_idx - kMaxLibSamplerate - 1]);

        const auto ds_waveform = downsampleWaveform(downsampling_src, ratio);
        if (ds_waveform.empty())
          throw std::runtime_error("Downsampled failed");

        computeMQValues(ds_waveform, waveform_idx, mipmap_idx);
      }
    }
  }
}

std::unique_ptr<WavetableData> WavetableData::build(Span<const float> waveforms,
                                                    int num_waveforms,
                                                    float samplerate,
                                                    float mipmap_ratio) {
  if (waveforms.empty())
    return nullptr;
  if (num_waveforms <= 0)
    return nullptr;
  if (samplerate <= 0.F)
    return nullptr;

  // The waveforms span must contains waveforms of the same length
  if (waveforms.size() % num_waveforms != 0)
    return nullptr;

  const auto waveform_len = waveforms.size() / num_waveforms;
  if (!maths::isPowerOfTwo(static_cast<int>(waveform_len)))
    return nullptr;

  // can't use make_unique because ctor is private
  try {
    return std::unique_ptr<WavetableData>(
        new WavetableData(waveforms, num_waveforms, samplerate, mipmap_ratio));
  } catch (std::runtime_error&) { return nullptr; }
}

void WavetableData::updateSamplerate(float samplerate) {
  // Recompute the mipmap scale
  computeMipMapScale(waveformLen(0), samplerate);
}

//==============================================================================
[[nodiscard]] WavetableData::MipMapIndices WavetableData::findMipMapIndices(
    float abs_phase_diff) const noexcept {
  assert(abs_phase_diff > 0.F);
  // TODO: optimize the search, this could be done by giving a hint for the
  // search, like the last index and the phase diff diff
  auto i = 0;
  while (i < mipmap_scale_.size() - 1) {
    if (abs_phase_diff < mipmap_scale_[i])
      break;

    ++i;
  }

  if (i == mipmap_scale_.size() - 1) {
    // Reached last index, no crossfade
    return std::make_tuple(i, 1.F, i + 1, 0.F);
  }

  auto threshold = mipmap_ratio_ * mipmap_scale_[i];

  if (abs_phase_diff < threshold) {
    // Below threshold, we don't crossfade
    return std::make_tuple(i, 1.F, i + 1, 0.F);
  }

  // Above threshold, starting crossfade
  auto a           = 1.0 / (mipmap_scale_[i] - threshold);
  auto b           = -threshold * a;
  auto factor_next = a * abs_phase_diff + b;
  auto factor_crt  = 1.0 - factor_next;

  return std::make_tuple(i, factor_crt, i + 1, factor_next);
}

// [[nodiscard]] std::vector<float> WavetableData::computeMipMapFrequencies(
//     float samplerate) const {
//   auto freqs = std::vector(mipmap_scale_);

//   for (auto& freq : freqs) {
//     freq *= samplerate;
//   }

//   return freqs;
// }

//==============================================================================
int WavetableData::computeNumMipMapTables(int waveform_len) {
  return maths::floor(std::log2(static_cast<float>(waveform_len) / 4.F)) + 1;
}

void WavetableData::computeMipMapScale(int waveform_len, float samplerate) {
  const auto start = samplerate / static_cast<float>(waveform_len) * 2.F;
  const auto num   = computeNumMipMapTables(waveform_len);

  // Compute the mipmap scale
  mipmap_scale_.resize(num);
  for (auto&& [i, val] : iter::enumerate(mipmap_scale_)) {
    val = start * static_cast<float>(std::pow(2.F, i)) / samplerate;
  }
}

void WavetableData::computeMQValues(Span<const float> waveform,
                                    int waveform_idx, int mipmap_idx) {
  assert(!m_.empty());
  assert(q_.size() == numWaveforms());
  assert(m_diff_.size() == numWaveforms());
  assert(q_diff_.size() == numWaveforms());
  assert(m_[0].size() == numMipMapTables());
  assert(q_[0].size() == numMipMapTables());
  assert(m_diff_[0].size() == numMipMapTables());
  assert(q_diff_[0].size() == numMipMapTables());
  assert(!phases_.empty());

  const auto waveform_len = static_cast<int>(waveform.size());
  auto& m_vec             = m_[waveform_idx][mipmap_idx];
  auto& q_vec             = q_[waveform_idx][mipmap_idx];
  auto& m_diff_vec        = m_diff_[waveform_idx][mipmap_idx];
  auto& q_diff_vec        = q_diff_[waveform_idx][mipmap_idx];

  // Resize vectors
  m_vec.resize(waveform.size());
  q_vec.resize(waveform.size());
  m_diff_vec.resize(waveform.size());
  q_diff_vec.resize(waveform.size());

  auto phase_vec = phases(mipmap_idx);

  // Compute M & Q values
  for (auto i : iter::range(waveform_len)) {
    auto y0 = waveform[i];
    auto y1 = waveform[(i + 1) % waveform_len];
    auto x0 = phase_vec[i];
    auto x1 = phase_vec[i + 1];

    m_vec[i] = computeM(x0, x1, y0, y1);
    q_vec[i] = computeQ(x0, x1, y0, y1);
  }

  // Compute M & Q diff
  for (auto i : iter::range(waveform_len)) {
    m_diff_vec[i] = m_vec[(i + 1) % waveform_len] - m_vec[i];
    q_diff_vec[i] = q_vec[(i + 1) % waveform_len] - q_vec[i];
  }
  q_diff_vec[waveform_len - 1] -= m_vec[0];
}

void WavetableData::computePhaseVector(int waveform_len, int mipmap_idx) {
  assert(!phases_.empty());
  phases_[mipmap_idx].resize(waveform_len + 1);

  for (auto&& [i, val] : iter::enumerate(phases_[mipmap_idx])) {
    val = static_cast<float>(i) / static_cast<float>(waveform_len);
  }
}

std::vector<float> WavetableData::downsampleWaveform(Span<const float> waveform,
                                                     int ratio) {
  constexpr auto kRepetitions = 5;
  static_assert(maths::isOdd(kRepetitions));
  const auto in_size  = static_cast<int>(waveform.size());
  const auto out_size = in_size / ratio;

  const auto src_quality = SRC_SINC_BEST_QUALITY;

  auto in_multiple  = std::vector<float>(in_size * kRepetitions);
  auto out_multiple = std::vector<float>(out_size * kRepetitions);

  // Copy the waveform multiple times
  for (auto i : iter::range(kRepetitions)) {
    std::copy(waveform.begin(), waveform.end(),
              in_multiple.begin() + i * in_size);
  }

  // Downsample the multiple waveforms
  auto src_data = SRC_DATA{in_multiple.data(),
                           out_multiple.data(),
                           static_cast<int64_t>(in_multiple.size()),
                           static_cast<int64_t>(out_multiple.size()),
                           0,
                           0,
                           0,
                           1. / static_cast<double>(ratio)};
  auto src_err  = src_simple(&src_data, src_quality, 1);
  if (src_err != 0) {
    std::cerr << src_strerror(src_err) << std::endl;
    return std::vector<float>();
  }
  if (src_data.input_frames_used != in_multiple.size() ||
      src_data.output_frames_gen != out_multiple.size()) {
    return std::vector<float>();
  }

  // Copy the center of the output to the result
  auto result       = std::vector<float>(out_size);
  auto start_offset = out_size * (kRepetitions / 2);
  auto end_offset   = out_size * (kRepetitions / 2 + 1);
  std::copy(out_multiple.begin() + start_offset,
            out_multiple.begin() + end_offset, result.begin());

  return result;
}

}  // namespace adwt