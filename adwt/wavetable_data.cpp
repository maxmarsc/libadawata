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
#include <cppitertools/zip.hpp>
#include <iostream>
#include <numeric>

#include "maths.hpp"

namespace adwt {

inline float computeM(double x0, double x1, double y0, double y1) {
  return static_cast<float>((y1 - y0) / (x1 - x0));
}

inline float computeQ(double x0, double x1, double y0, double y1) {
  return static_cast<float>((y0 * (x1 - x0) - x0 * (y1 - y0)) / (x1 - x0));
}

//==============================================================================
WavetableData::WavetableData(Span<const float> waveforms, int num_waveforms,
                             float samplerate, int crossfade_samples,
                             int og_waveform_len)
    : cross_fader_(crossfade_samples, og_waveform_len, samplerate) {
  assert(og_waveform_len * num_waveforms == waveforms.size());

  // Resize the vectors to be sure then can hold enough waveforms
  m_.resize(num_waveforms);
  q_.resize(num_waveforms);
  m_diff_.resize(num_waveforms);
  q_diff_.resize(num_waveforms);

  // Compute the phase points vectors
  computePhaseVectors(og_waveform_len);

  // We iterate over each waveform
  for (auto waveform_idx : iter::range(num_waveforms)) {
    auto og_waveform =
        waveforms.subspan(waveform_idx * og_waveform_len, og_waveform_len);

    // Compute MQ for the og waveform
    m_[waveform_idx].resize(numMipMapTables());
    q_[waveform_idx].resize(numMipMapTables());
    m_diff_[waveform_idx].resize(numMipMapTables());
    q_diff_[waveform_idx].resize(numMipMapTables());
    computeMQVector(og_waveform, waveform_idx, 0);

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

        computeMQVector(ds_waveform, waveform_idx, mipmap_idx);
      } else {
        auto ratio            = 256;
        auto downsampling_src = Span<const float>(
            intermediate_tables[mipmap_idx - kMaxLibSamplerate - 1]);

        const auto ds_waveform = downsampleWaveform(downsampling_src, ratio);
        if (ds_waveform.empty())
          throw std::runtime_error("Downsampled failed");

        computeMQVector(ds_waveform, waveform_idx, mipmap_idx);
      }
    }
  }
}

std::unique_ptr<WavetableData> WavetableData::build(
    Span<const float> waveforms, int num_waveforms, float samplerate,
    float crossfade_duration_s) {
  if (waveforms.empty())
    return nullptr;
  if (num_waveforms <= 0)
    return nullptr;
  if (samplerate <= 0.F)
    return nullptr;

  // The waveforms span must contains waveforms of the same length
  if (waveforms.size() % num_waveforms != 0)
    return nullptr;

  const auto waveform_len = static_cast<int>(waveforms.size()) / num_waveforms;
  if (!maths::isPowerOfTwo(static_cast<int>(waveform_len)))
    return nullptr;

  const auto crossfade_samples =
      static_cast<int>(crossfade_duration_s * samplerate);

  // can't use make_unique because ctor is private
  try {
    return std::unique_ptr<WavetableData>(new WavetableData(
        waveforms, num_waveforms, samplerate, crossfade_samples, waveform_len));
  } catch (std::runtime_error&) { return nullptr; }
}

void WavetableData::updateSamplerate(float samplerate,
                                     float crossfade_duration_s) {
  auto og_waveforms = rebuildWaveforms();
  auto new_data =
      build(og_waveforms, numWaveforms(), samplerate, crossfade_duration_s);
  std::swap(*this, *new_data);
}

//==============================================================================
void WavetableData::computeMQVector(Span<const float> waveform,
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

void WavetableData::computePhaseVectors(int og_waveform_len) {
  const auto num_mm_tables = numMipMapTables();
  const auto prev_num      = static_cast<int>(phases_.size());

  if (num_mm_tables > prev_num) {
    // Missing some phase vectors
    phases_.resize(num_mm_tables);

    // We compute every missing phase vector
    for (const auto mm_idx : iter::range(prev_num, num_mm_tables)) {
      const auto waveform_len = og_waveform_len / (1 << mm_idx);
      computePhaseVector(waveform_len, mm_idx);
    }

  } else if (num_mm_tables < prev_num) {
    // New mm count is lower, we can remove some mm tables
    phases_.resize(num_mm_tables);
  }
}

void WavetableData::computePhaseVector(int waveform_len, int mipmap_idx) {
  assert(!phases_.empty());
  phases_[mipmap_idx].resize(waveform_len + 1);

  for (auto&& [i, val] : iter::enumerate(phases_[mipmap_idx])) {
    val = static_cast<float>(i) / static_cast<float>(waveform_len);
  }
}

std::vector<float> WavetableData::rebuildWaveforms() {
  assert(!phases_.empty());
  const auto num_waveforms = numWaveforms();
  const auto waveform_size = waveformLen(0);
  auto ret                 = std::vector<float>(num_waveforms * waveform_size);

  for (const auto waveform_idx : iter::range(num_waveforms)) {
    auto start         = ret.begin() + waveform_idx * waveform_size;
    auto end           = start + waveform_size;
    auto waveform_span = Span<float>(start, end);

    auto m_span = m(waveform_idx, 0);
    auto q_span = q(waveform_idx, 0);
    auto phase  = phases(0);

    for (auto&& [m, q, x, y] :
         iter::zip(m_span, q_span, phase, waveform_span)) {
      y = m * x + q;
    }
  }

  return ret;
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