/*
* waveform_data.hpp
* Created by maxmarsc, 25/09/2023
*/

#pragma once

#include <cassert>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

#include "span.hpp"

namespace adwt {

class WaveformData {
  //============================================================================
  WaveformData(Span<const float> waveforms, int num_waveforms, int samplerate,
               float mipmap_ratio);

 public:
  using MipMapIndexes = std::tuple<int, float, int, float>;

  static std::unique_ptr<WaveformData> build(Span<const float> waveforms,
                                             int num_waveforms, int samplerate,
                                             float mipmap_ratio = 0.98F);

  //============================================================================
  [[nodiscard]] inline Span<const float> m(int waveform,
                                           int mipmap) const noexcept {
    return m_[waveform][mipmap];
  }
  [[nodiscard]] inline Span<const float> q(int waveform,
                                           int mipmap) const noexcept {
    return q_[waveform][mipmap];
  }
  [[nodiscard]] inline Span<const float> mDiff(int waveform,
                                               int mipmap) const noexcept {
    return m_diff_[waveform][mipmap];
  }
  [[nodiscard]] inline Span<const float> qDiff(int waveform,
                                               int mipmap) const noexcept {
    return q_diff_[waveform][mipmap];
  }
  [[nodiscard]] inline Span<const float> phases(int mipmap) const noexcept {
    return phases_[mipmap];
  }
  [[nodiscard]] inline int numWaveforms() const noexcept {
    return static_cast<int>(m_.size());
  }
  [[nodiscard]] inline int numMipMapTables() const noexcept {
    return static_cast<int>(mipmap_scale_.size());
  }
  [[nodiscard]] inline int waveformLen(int mipmap) const noexcept {
    return static_cast<int>(m_[0][mipmap].size());
  }
  [[nodiscard]] inline std::tuple<float, float> minMaxPhaseDiffRatio()
      const noexcept {
    return std::make_tuple(mipmap_ratio_, 1.F / mipmap_ratio_);
  };

  [[nodiscard]] MipMapIndexes findMipMapIndexes(
      float phase_diff) const noexcept;

  //==============================================================================
 private:
  void computeMipMapScale(int waveform_len, float samplerate);
  void computeMQValues(Span<const float> waveform, int waveform_idx,
                       int mipmap_idx);
  void computePhaseVector(int waveform_len, int mipmap_idx);

  static std::vector<float> downsampleWaveform(Span<const float> waveform,
                                               int ratio);

  //=============================================================================
  std::vector<std::vector<std::vector<float>>> m_;
  std::vector<std::vector<std::vector<float>>> q_;
  std::vector<std::vector<std::vector<float>>> m_diff_;
  std::vector<std::vector<std::vector<float>>> q_diff_;
  std::vector<std::vector<float>> phases_;
  std::vector<float> mipmap_scale_;
  const float mipmap_ratio_;
};

}  // namespace adwt