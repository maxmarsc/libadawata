/*
* wavetable_data.hpp
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

#pragma once

#include <cassert>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

#include "span.hpp"

namespace adwt {

/**
 * @brief Compute and holds the data of a wavetable to be used by an oscillator
 *
 * The interface of this class allows the creation of a wavetable through an unique
 * ptr, which can then be passed to an oscillator instance.
 */
class WavetableData {
  //============================================================================
  WavetableData(Span<const float> waveforms, int num_waveforms,
                float samplerate, float mipmap_ratio);

 public:
  /**
   * @brief 4-values tuples on how to cross-fade
   * 0: The lowest mipmap idx to use
   * 1: The highest mipmap idx to use
   * 2: The gain [0:1] to apply to the mipmap entry of 0
   * 3: The gain [0:1] to apply to the mipmap entry of 1
   * 
   */
  using MipMapIndices = std::tuple<int, float, int, float>;

  /**
   * @brief Builds a new WavetableData instance for a given wavetable
   * 
   * @note When using a wavetable with non negligeable harmonics differences
   * between waveforms in the wavetable, the caller is responsible for any cross-fading
   * necessary to attenuate audio "clicks" due to jumps in harmonics when updating
   * which waveform to use in the wavetable.
   *
   * @param waveforms A span containing all the waveforms in the wavetable, stored
   * contiguously. They must all be of the same size, which must be a power of 
   * two (maximum 256²)
   * @param num_waveforms The number of waveforms in the wavetable
   * @param samplerate The samplerate at which this waveform is going to be played
   * (needed for mipmapping cross-fading settings)
   * @param mipmap_ratio Ratio (0:1) of each octave that will be cross-faded with
   * the lowest octave
   */
  static std::unique_ptr<WavetableData> build(Span<const float> waveforms,
                                              int num_waveforms,
                                              float samplerate,
                                              float mipmap_ratio = 0.98F);

  /**
   * @brief Updates the samplerate of the wavetable by recomputing mipmap transition
   * points
   */
  void updateSamplerate(float samplerate);

  //============================================================================
  /**
   * @brief A span of the M values
   * 
   * @param waveform The index of the waveform to get M values from
   * @param mipmap The index of the mipmap entry to get M values from
   */
  [[nodiscard]] inline Span<const float> m(int waveform,
                                           int mipmap) const noexcept {
    return m_[waveform][mipmap];
  }
  /**
   * @brief A span of the Q values
   * 
   * @param waveform The index of the waveform to get Q values from
   * @param mipmap The index of the mipmap entry to get Q values from
   */
  [[nodiscard]] inline Span<const float> q(int waveform,
                                           int mipmap) const noexcept {
    return q_[waveform][mipmap];
  }
  /**
   * @brief A span of the M_diff values (M_diff[i] = M[i] - M[i-1])
   * 
   * @param waveform The index of the waveform to get M_diff values from
   * @param mipmap The index of the mipmap entry to get M_diff values from
   */
  [[nodiscard]] inline Span<const float> mDiff(int waveform,
                                               int mipmap) const noexcept {
    return m_diff_[waveform][mipmap];
  }
  /**
   * @brief A span of the Q_diff values (Q_diff[i] = Q[i] - Q[i-1])
   * 
   * @param waveform The index of the waveform to get Q values from
   * @param mipmap The index of the mipmap entry to get Q values from
   */
  [[nodiscard]] inline Span<const float> qDiff(int waveform,
                                               int mipmap) const noexcept {
    return q_diff_[waveform][mipmap];
  }
  /**
   * @brief A span of the phase of each sample in the waveform
   * 
   * @param mipmap The index of the mipmap entry to get phases values from
   */
  [[nodiscard]] inline Span<const float> phases(int mipmap) const noexcept {
    return phases_[mipmap];
  }
  /**
   * @brief Number of waveforms in the wavetable
   */
  [[nodiscard]] inline int numWaveforms() const noexcept {
    return static_cast<int>(m_.size());
  }
  /**
   * @brief Number of mipmap entry in the mipmap table. This will depends on the
   * size of the waveform and the samplerate
   */
  [[nodiscard]] inline int numMipMapTables() const noexcept {
    return static_cast<int>(mipmap_scale_.size());
  }
  /**
   * @brief Size of the mipmap entry of the given index
   */
  [[nodiscard]] inline int waveformLen(int mipmap) const noexcept {
    return static_cast<int>(m_[0][mipmap].size());
  }
  /**
   * @brief Ratios to help compute an estimation of the frequency variation limitation
   * induced by the cross-fading implementation.
   *
   * This function returns [min, max]. Given your previous phase_diff value :
   *  - min * phase_diff : the next minimum phase_diff you should be able to use
   * without introducing audio "clicks"
   * - max * phase_diff : the next maximum phase_diff you should be able to use
   * without introducing audio "clicks"
   */
  [[nodiscard]] inline std::tuple<float, float> minMaxPhaseDiffRatio()
      const noexcept {
    return std::make_tuple(mipmap_ratio_, 1.F / mipmap_ratio_);
  };

  /**
   * @brief Returns the @ref MipMapIndices tuple for the given phase diff
   */
  [[nodiscard]] MipMapIndices findMipMapIndices(
      float phase_diff) const noexcept;

  // /**
  //  * @brief Computes the corresponding frequency of each mipmap transition point
  //  *
  //  * @param samplerate The samplerate for which the mipmap table was computed for
  //  * @return std::vector<float> A vector of frequency values, should be ordered
  //  */
  // [[nodiscard]] std::vector<float> computeMipMapFrequencies(
  //     float samplerate) const;
  /**
   * @brief Returns the mipmap table of phase point. Each phase point represents a 
   * transition point between two mipmap tables
   */
  [[nodiscard]] inline Span<const float> mipMapTable() const noexcept {
    return mipmap_scale_;
  }

  //==============================================================================
 private:
  static int computeNumMipMapTables(int waveform_len);
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