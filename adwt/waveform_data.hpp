/*
* waveform_data.hpp
* Created by maxmarsc, 25/09/2023
*/

#pragma once

#include <cassert>
#include <cmath>
#include <memory>
#include <span>
#include <tuple>
#include <vector>

namespace adwt {

class WaveformData {
  //============================================================================
  WaveformData(std::span<const float> waveforms, int num_waveforms,
               int samplerate);

 public:
  using MipMapIndexes = std::tuple<int, float, int, float>;

  static std::unique_ptr<WaveformData> build(std::span<const float> waveforms,
                                             int num_waveforms, int samplerate);

  //============================================================================
  [[nodiscard]] inline std::span<const float> m(int waveform,
                                                int mipmap) const noexcept {
    return m_[waveform][mipmap];
  }
  [[nodiscard]] inline std::span<const float> q(int waveform,
                                                int mipmap) const noexcept {
    return q_[waveform][mipmap];
  }
  [[nodiscard]] inline std::span<const float> mDiff(int waveform,
                                                    int mipmap) const noexcept {
    return m_diff_[waveform][mipmap];
  }
  [[nodiscard]] inline std::span<const float> qDiff(int waveform,
                                                    int mipmap) const noexcept {
    return q_diff_[waveform][mipmap];
  }
  [[nodiscard]] inline std::span<const float> phases(
      int mipmap) const noexcept {
    return phases_[mipmap];
  }
  [[nodiscard]] inline int numWaveforms() const noexcept {
    return static_cast<int>(m_.size());
  }
  [[nodiscard]] inline int numMipMapTables() const noexcept {
    return static_cast<int>(mipmap_scale_.size());
  }

  [[nodiscard]] MipMapIndexes findMipMapIndexes(
      float phase_diff) const noexcept;

  //==============================================================================
 private:
  void computeMipMapScale(int waveform_len, int samplerate);
  void computeMQValues(std::span<const float> waveform, int waveform_idx,
                       int mipmap_idx);
  void computePhaseVector(int waveform_len, int mipmap_idx);

  static std::vector<float> downsampleWaveform(std::span<const float> waveform,
                                               int ratio);

  //=============================================================================
  std::vector<std::vector<std::vector<float>>> m_;
  std::vector<std::vector<std::vector<float>>> q_;
  std::vector<std::vector<std::vector<float>>> m_diff_;
  std::vector<std::vector<std::vector<float>>> q_diff_;
  std::vector<std::vector<float>> phases_;
  std::vector<float> mipmap_scale_;
};

}  // namespace adwt