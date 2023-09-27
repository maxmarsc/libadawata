/*
* waveform_data.cpp
* Created by maxmarsc, 25/09/2023
*/

#include "waveform_data.hpp"

#include <cmath>

#include <soxr.h>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>

#include "maths.hpp"

namespace adwt {

inline float computeM(float x0, float x1, float y0, float y1) {
  return (y1 - y0) / (x1 - x0);
}

inline float computeQ(float x0, float x1, float y0, float y1) {
  return (y0 * (x1 - x0) - x0 * (y1 - y0)) / (x1 - x0);
}

//==============================================================================
WaveformData::WaveformData(std::span<const float> waveforms, int num_waveforms,
                           int samplerate) {
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
    const auto waveform_len = og_waveform_len / (2 << mipmap_idx);
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

    // Compute MQ for each mipmap entry
    for (auto mipmap_idx : iter::range(1, numMipMapTables())) {
      auto ratio             = 2 << mipmap_idx;
      const auto ds_waveform = downsampleWaveform(og_waveform, ratio);
      if (ds_waveform.empty())
        throw std::runtime_error("Downsampled failed");

      computeMQValues(ds_waveform, waveform_idx, mipmap_idx);
    }
  }
}

std::unique_ptr<WaveformData> WaveformData::build(
    std::span<const float> waveforms, int num_waveforms, int samplerate) {
  // The waveforms span must contains waveforms of the same length
  if (waveforms.size() % num_waveforms != 0)
    return nullptr;

  // can't use make_unique because ctor is private
  try {
    return std::unique_ptr<WaveformData>(
        new WaveformData(waveforms, num_waveforms, samplerate));
  } catch (std::runtime_error&) { return nullptr; }
}

//==============================================================================
[[nodiscard]] WaveformData::MipMapIndexes WaveformData::findMipMapIndexes(
    float phase_diff) const noexcept {
  assert(phase_diff > 0);
  constexpr auto kThresholdRatio = 0.99F;
  constexpr auto kTest           = 0.1F;

  // TODO: optimize the search, this could be done by giving a hint for the
  // search, like the last index and the phase diff diff
  auto i = 0;
  while (i < mipmap_scale_.size()) {
    if (phase_diff < mipmap_scale_[i])
      break;

    ++i;
  }

  if (i == mipmap_scale_.size()) {
    // Reached last index, no crossfade
    return std::make_tuple(i, 1.F, i + 1, 0.F);
  }

  auto threshold = kThresholdRatio * mipmap_scale_[i];

  if (phase_diff < threshold) {
    // Below threshold, we don't crossfade
    return std::make_tuple(i, 1.F, i + 1, 0.F);
  }

  // Above threshold, starting crossfade
  auto a           = 1.0 / (mipmap_scale_[i] - threshold);
  auto b           = -threshold * a;
  auto factor_next = a * phase_diff + b;
  auto factor_crt  = 1.0 - factor_next;

  return std::make_tuple(i, factor_crt, i + 1, factor_next);
}

//==============================================================================
void WaveformData::computeMipMapScale(int waveform_len, int samplerate) {
  auto start =
      static_cast<float>(samplerate) / static_cast<float>(waveform_len) * 2.F;
  auto num =
      maths::floor(std::log2(static_cast<float>(samplerate) / 2.F / start));

  // Compute the mipmap scale
  mipmap_scale_.resize(num);
  for (auto&& [i, val] : iter::enumerate(mipmap_scale_)) {
    val = start * static_cast<float>(std::pow(2.F, i));
  }
}

void WaveformData::computeMQValues(std::span<const float> waveform,
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
    auto x1 = phase_vec[(i + 1) % waveform_len];

    m_vec[i] = computeM(x0, x1, y0, y1);
    q_vec[i] = computeQ(x0, x1, y0, y1);
  }

  // Compute M & Q diff
  for (auto i : iter::range(waveform_len)) {
    m_diff_vec[i] = m_vec[(i + 1) % waveform_len] - m_vec[i];
    q_diff_vec[i] = q_vec[(i + 1) % waveform_len] - q_vec[i];
  }
}

void WaveformData::computePhaseVector(int waveform_len, int mipmap_idx) {
  assert(!phases_.empty());
  phases_[mipmap_idx].resize(waveform_len + 1);

  for (auto&& [i, val] : iter::enumerate(phases_[mipmap_idx])) {
    val = static_cast<float>(i) / static_cast<float>(waveform_len);
  }
}

std::vector<float> WaveformData::downsampleWaveform(
    std::span<const float> waveform, int ratio) {
  constexpr auto kRepetitions = 3;
  static_assert(maths::isOdd(kRepetitions));
  const auto in_size  = static_cast<int>(waveform.size());
  const auto out_size = in_size / ratio;

  const soxr_quality_spec_t soxr_quality{SOXR_VHQ};

  // auto result       = std::vector<float>(out_size);
  auto out_multiple = std::vector<float>(out_size * kRepetitions);
  auto in_multiple  = std::vector<float>(waveform.size() * kRepetitions);

  // Copy the waveform multiple times
  for (auto i : iter::range(kRepetitions)) {
    std::copy(waveform.begin(), waveform.end(),
              in_multiple.begin() + i * in_size);
  }

  // Downsample the multiple waveforms
  size_t idone = 0;
  size_t odone = 0;
  soxr_oneshot(ratio, 1.0, 1, in_multiple.data(), in_size * kRepetitions,
               &idone, out_multiple.data(), out_size * kRepetitions, &odone,
               nullptr, &soxr_quality, nullptr);
  if (idone != in_size * kRepetitions || odone != out_size * kRepetitions)
    return std::vector<float>();

  // Copy the center of the output to the result
  auto result       = std::vector<float>(out_size);
  auto start_offset = out_size * (kRepetitions / 2);
  auto end_offset   = out_size * (kRepetitions / 2 + 1);
  std::copy(out_multiple.begin() + start_offset,
            out_multiple.begin() + end_offset, result.begin());

  return result;
}

}  // namespace adwt