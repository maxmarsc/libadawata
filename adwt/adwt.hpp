/*
* adwt.hpp
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

#if defined(__CLANGD__) && defined(__aarch64__)
#include "extras/clangd_arm_neon.h"
#endif

#include <iostream>
#include <memory>
#include <numeric>

#include <cppitertools/range.hpp>
#include <cppitertools/zip.hpp>
#include <xsimd/xsimd.hpp>

#include "direction.hpp"
#include "filter_type.hpp"
#include "maths.hpp"
#include "span.hpp"
#include "wavetable_data.hpp"

namespace xs = xsimd;

namespace adwt {

/**
 * @brief The main class of the library. This Oscillator class implement real-time
 * ADAA-IIR.
 *
 * This implementation supports waveform with sizes of power of two, and should be
 * feeded with phase values within [0:1)
 *
 * It has limitations on how fast the frequency of the signal can vary. Too fast
 * of a variation would not make the implementation crash, but might introduce 
 * audio "clicks" due to unattenuated jumps in harmonics.
 *
 * Anti-Derivative Anti-Aliasing (ADAA) is an "new" anti-aliasing technique. This
 * implementation is based on the work of Leonardo Gabrielli, Stefano D'Angelo,
 * Pier Paolo La Pastina and Stefano Squartini : 
 *
 * "Antiderivative Antialiasing for Arbitrary Waveform Generation" - August 2022
 * https://www.researchgate.net/publication/362628103_Antiderivative_Antialiasing_for_Arbitrary_Waveform_Generation
 *
 * @tparam Ftype The type of anti-aliasing filter to use. See @file filter_type.hpp
 */
template <FilterType Ftype>
class Oscillator {
  //============================================================================
  //                                INTERFACE
  //============================================================================
 public:
  /**
  * @brief Default constructor. The user must call @ref init() before use, in order to
  * load a waveform
  */
  Oscillator();

  //============================================================================
  /**
   * @brief Load a waveform in the oscillator.
   *
   * @note This should only be called once. To change a loaded waveform the user
   * shall call @ref swapWavetable
   * 
   * @param waveform_data A WavetableData object to load. The Oscillator object will
   * take the handle of the pointer.
   * @param init_state A optionnal init tuple to set both previous phase and phase_diff
   * values (because this is a recursive algorithm). Once initialized, the user
   * can reset again these values at any time by calling @ref resetInternals()
   * @return int 0 on success
   */
  [[nodiscard]] int init(
      std::unique_ptr<WavetableData>&& wavetable_data,
      std::tuple<float, float> init_state = std::make_tuple(0.F, 0.4F));

  /**
   * @brief Swap the current loaded waveform with a new one.
   * 
   * @param waveform_data A WavetableData object to load. The Oscillator object will
   * take the handle of the pointer.
   * @param init_state A optionnal init tuple to set both previous phase and phase_diff
   * values (because this is a recursive algorithm). Once initialized, the user
   * can reset again these values at any time by calling @ref resetInternals()
   * @return std::unique_ptr<WavetableData> 
   */
  [[nodiscard]] std::unique_ptr<WavetableData> swapWavetable(
      std::unique_ptr<WavetableData>&& wavetable_data,
      std::tuple<float, float> init_state = std::make_tuple(0.F, 0.4F));

  /**
   * @brief Reset the previous phase and phase_diff values. Usefull when the user
   * want to suddenly change the frequency of the signal. The caller is responsible
   * for any necessary cross-fading.
   * 
   * @param init_phase The new previous phase value to set
   * @param init_phase_diff The new previous phase_diff value to set
   */
  void resetInternals(float init_phase, float init_phase_diff);

  /**
   * @brief Updates the samplerate of the wavetable by recomputing mipmap transition
   * points
   *
   * @note This will rebuild the original waveforms from the M & Q points,
   * which can causes a loss of precision, up to 4e-3F (for a randomized [-1:1] waveform)
   * If you want a better accuraccy you'd better create a new WavetableData object
   */
  inline void updateSamplerate(float samplerate) {
    assert(wavetable_ != nullptr);
    wavetable_->updateSamplerate(samplerate);
  }

  //============================================================================
  /**
   * @brief Generate a waveform signal according to the given phase values
   * 
   * @tparam Dir Can be set to kForward if you know your signal will only be played
   * in the forward direction. It can slightly improve speed and binary size
   * @param phases A span of phases values of the waveform samples to produce
   * @param output A span to store the produced waveform samples. Its size must
   * match the phases span's size.
   */
  template <Direction Dir = Direction::kBidirectionnal>
  inline void process(Span<const float> phases, Span<float> output) noexcept {
    if constexpr (Dir == Direction::kForward) {
      processFwd(phases, output);
    } else {
      processBi(phases, output);
    }
  }

  //============================================================================
  /**
   * @brief The number of waveforms in the loaded wavetable
   */
  [[nodiscard]] inline int numWaveforms() const noexcept {
    assert(wavetable_ != nullptr);
    return wavetable_->numWaveforms();
  }
  /**
   * @brief The index of the current waveform in the loaded wavetable
   */
  [[nodiscard]] inline int crtWaveform() const noexcept {
    assert(wavetable_ != nullptr);
    return crt_waveform_;
  }
  /**
   * @brief Set the waveform to use from the loaded wavetable
   * @note The caller is responsible for any cross-fading necessary to attenuate
   * audio "clicks" due to jumps in harmonics.
   */
  inline void setWaveform(int waveform_idx) noexcept {
    assert(wavetable_ != nullptr);
    crt_waveform_ = waveform_idx;
  }
  /**
   * @brief The phase of the previously generated sample
   */
  [[nodiscard]] inline float prevPhase() const noexcept {
    assert(wavetable_ != nullptr);
    return prev_phase_;
  }
  /**
   * @brief The phase diff betwen the previously generated sample and the one
   * before
   */
  [[nodiscard]] inline float prevPhaseDiff() const noexcept {
    assert(wavetable_ != nullptr);
    return prev_phase_diff_;
  }

  //============================================================================
  //                         IMPLEMENTATION DETAILS
  //============================================================================
 private:
  using CpxBatch                   = xs::batch<std::complex<float>>;
  static constexpr auto kBatchSize = CpxBatch::size;

  /**
   * @brief If the number of complex values to compute modulo the size of a SIMD
   * complex batch is above this threshold, we will use a SIMD batch to compute
   * these complex values. If less or equal, we will use a scalar implementation.
   */
  static constexpr auto kBatchThreshold = 1;

  /**
   * @brief Computes the upper number of coeffs to actually use in order to
   * perfectly fits SIMD registers
   */
  static constexpr std::size_t computeUpperNumCoeffs() {
    constexpr auto kNumCoeffs = numCoeffs<Ftype>();
    static_assert(kBatchSize != 0);
    constexpr auto kNumBatch = maths::floor(static_cast<float>(kNumCoeffs) /
                                            static_cast<float>(kBatchSize));
    if constexpr (kNumBatch == 0) {
      return kBatchSize;
    } else {
      if (kNumCoeffs % kBatchSize == 0) {
        return kNumBatch * kBatchSize;
      }
      return (kNumBatch + 1) * kBatchSize;
    }
  }

  static constexpr auto kNumCoeffs      = numCoeffs<Ftype>();
  static constexpr auto kUpperNumCoeffs = computeUpperNumCoeffs();
  static constexpr auto kAligment       = CpxBatch::arch_type::alignment();
  static constexpr auto kPartialSize    = kNumCoeffs % kBatchSize;
  static constexpr auto kWholeSize      = kNumCoeffs - kPartialSize;

  //============================================================================
  /**
   * @brief Compute the first part of the I member of the algorithm.
   * 
   * @param aligned_array Where to store the result. The content will be overwritten
   * @param mipmap_idx The mipmap idx to use for computation
   * @param idx_prev_bound Should correspond to the first waveform sample index
   * after the previously generated output sample
   * @param idx_next_bound Should correspond to the first waveform sample index
   * after the newly generated output sample
   * @param phase_diff The phase difference between the new output sample and the
   * previous output sample
   * @param prev_phase_red_bar TODO: find a proper way to explain
   * @param phase_red_bar TODO: find a proper way to explain
   */
  void computeI0(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int idx_prev_bound, int idx_next_bound, float phase_diff,
      float prev_phase_red_bar, float phase_red_bar) const noexcept;

  /**
   * @brief Computes the forward version of the second part of the I member.
   * 
   * @param aligned_array Where to add the result. The caller must make sure
   * it contains the result of the previously computed I0 part
   * @param mipmap_idx The mipmap idx to use for computation
   * @param jmin_red Reduce version of the index jmin
   * @param jmax_p_red Reduce version of the index jmax + 1
   * @param phase_diff The phase difference between the new output sample and the
   * previous output sample
   * @param phase_red Reduce version of the new phase value
   */
  void computeISumFwd(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept;

  /**
   * @brief Computes the backward version of the second part of the I member.
   * 
   * @param aligned_array Where to add the result. The caller must make sure
   * it contains the result of the previously computed I0 part
   * @param mipmap_idx The mipmap idx to use for computation
   * @param jmin The index jmin
   * @param jmin_red Reduce version of the index jmin
   * @param jmax_p_red Reduce version of the index jmax + 1
   * @param phase_diff The phase difference between the new output sample and the
   * previous output sample
   * @param phase_red Reduce version of the new phase value
   */
  void computeISumBwd(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept;

  /**
   * @brief Computes the backward version of the I member
   * 
   * @param aligned_array Where to store the result. The content will be overwritten
   * @param mipmap_idx The mipmap idx to use for computation
   * @param jmin The index jmin
   * @param jmin_red Reduce version of the index jmin
   * @param jmax_p_red Reduce version of the index jmax + 1
   * @param phase_diff The phase difference between the new output sample and the
   * previous output sample
   * @param phase_red Reduce version of the new phase value
   */
  void computeBwdI(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept;

  /**
   * @brief Computes the forward version of the I member
   * 
   * @param aligned_array Where to store the result. The content will be overwritten
   * @param mipmap_idx The mipmap idx to use for computation
   * @param jmin_red Reduce version of the index jmin
   * @param jmax_p_red Reduce version of the index jmax + 1
   * @param phase_diff The phase difference between the new output sample and the
   * previous output sample
   * @param phase_red Reduce version of the new phase value
   */
  void computeFwdI(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept;

  /**
   * @brief Forward-only processing of a span of values
   */
  void processFwd(Span<const float> phases, Span<float> output) noexcept;

  /**
   * @brief Bidirectionnal processing of an entire span of values
   */
  void processBi(Span<const float> phases, Span<float> output) noexcept;

  /**
   * @brief Forward-only processing of a new output value
   * 
   * @param phase The phase of the new value
   * @param phase_diff The phase diff with the previous value
   * @param output Where to store the result
   */
  void processSampleFwd(float phase, float phase_diff, float& output) noexcept;

  /**
   * @brief Backward-only processing of a new output value
   * 
   * @param phase The phase of the new value
   * @param phase_diff The phase diff with the previous value
   * @param output Where to store the result
   */
  void processSampleBwd(float phase, float phase_diff, float& output) noexcept;
  //============================================================================
  static constexpr std::array<std::complex<float>, kUpperNumCoeffs> rArray() {
    std::array<std::complex<float>, kUpperNumCoeffs> ret{};
    constexpr auto kRArray = r<Ftype>();

    std::copy(kRArray.begin(), kRArray.end(), ret.begin());
    return ret;
  }

  static constexpr std::array<std::complex<float>, kUpperNumCoeffs> zArray() {
    std::array<std::complex<float>, kUpperNumCoeffs> ret{};
    constexpr auto kZArray = z<Ftype>();

    std::copy(kZArray.begin(), kZArray.end(), ret.begin());
    return ret;
  }

  static std::array<std::complex<float>, kUpperNumCoeffs> zPow2Array() {
    std::array<std::complex<float>, kUpperNumCoeffs> ret{};
    auto z_pow_2_array = zPow2<Ftype>();

    std::copy(z_pow_2_array.begin(), z_pow_2_array.end(), ret.begin());
    return ret;
  }

  static std::array<std::complex<float>, kUpperNumCoeffs> expZArray() {
    std::array<std::complex<float>, kUpperNumCoeffs> ret{};
    auto exp_z_array = zExp<Ftype>();

    std::copy(exp_z_array.begin(), exp_z_array.end(), ret.begin());
    return ret;
  }

  //============================================================================
  // The currently loaded wavetable
  std::unique_ptr<WavetableData> wavetable_;

  // Filter coefficients
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> r_array_;
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> z_array_;
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> z_pow2_array_;
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> exp_z_array_;

  // Storage for the recursive part of the algorithm
  std::array<std::complex<float>, kNumCoeffs> prev_cpx_y_array_{};
  float prev_phase_{};
  float prev_phase_red_{};
  float prev_phase_diff_{};
  int prev_mipmap_idx_{};
  int j_red_{};
  int prev_j_red_{};

  // Currently selected waveform in the wavetable
  int crt_waveform_{};
};

}  // namespace adwt