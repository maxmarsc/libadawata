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
  Oscillator()
      : wavetable_(nullptr),
        r_array_(rArray()),
        z_array_(zArray()),
        z_pow2_array_(zPow2Array()),
        exp_z_array_(expZArray()) {}

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
      std::tuple<float, float> init_state = std::make_tuple(0.F, 0.4F)) {
    if (wavetable_data == nullptr)
      return 1;
    wavetable_ = std::move(wavetable_data);
    resetInternals(std::get<0>(init_state), std::get<1>(init_state));
    crt_waveform_ = 0;
    return 0;
  }

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
      std::tuple<float, float> init_state = std::make_tuple(0.F, 0.4F)) {
    auto waveform_data_ptr = std::move(wavetable_data);
    if (waveform_data_ptr == nullptr)
      return nullptr;
    waveform_data_ptr.swap(wavetable_);
    resetInternals(std::get<0>(init_state), std::get<1>(init_state));
    crt_waveform_ = 0;
    return waveform_data_ptr;
  }

  /**
   * @brief Reset the previous phase and phase_diff values. Usefull when the user
   * want to suddenly change the frequency of the signal. The caller is responsible
   * for any necessary cross-fading.
   * 
   * @param init_phase The new previous phase value to set
   * @param init_phase_diff The new previous phase_diff value to set
   */
  void resetInternals(float init_phase, float init_phase_diff) {
    assert(wavetable_ != nullptr);
    prev_cpx_y_array_ = std::array<std::complex<float>, kNumCoeffs>{};
    prev_phase_       = init_phase;
    prev_phase_red_   = maths::reduce(init_phase, 1.F);
    prev_phase_diff_  = init_phase_diff;
    prev_mipmap_idx_ =
        std::get<0>(wavetable_->findMipMapIndices(std::fabs(init_phase_diff)));
    const auto waveform_len = wavetable_->waveformLen(prev_mipmap_idx_);

    if (init_phase_diff >= 0) {
      j_red_ = maths::floor(prev_phase_red_ * waveform_len);
    } else {
      j_red_ = maths::ceil(prev_phase_red_ * waveform_len);
    }
  }

  /**
   * @brief Updates the samplerate of the wavetable by recomputing mipmap transition
   * points
   */
  void updateSamplerate(float samplerate) {
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
  inline void process(Span<const float> phases, Span<float> output) {
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
    assert(wavetable_ != nullptr);
    return wavetable_->minMaxPhaseDiffRatio();
  };

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

 public:
 private:
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
  inline void computeI0(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int idx_prev_bound, int idx_next_bound, float phase_diff,
      float prev_phase_red_bar, float phase_red_bar) const noexcept {

    // Get the spans of m & q
#if !defined(ADWT_ENABLE_CXX20) & defined(NDEBUG)
    // gsl::span performs bound-checking on the [] operator
    const auto* m_span = wavetable_->m(crt_waveform_, mipmap_idx).data();
    const auto* q_span = wavetable_->q(crt_waveform_, mipmap_idx).data();
#else
    auto m_span     = wavetable_->m(crt_waveform_, mipmap_idx);
    auto q_span     = wavetable_->q(crt_waveform_, mipmap_idx);
#endif

    // Compute the recurrent parts of I_0
    const auto part_a = m_span[idx_prev_bound] * phase_diff;
    const auto part_b =
        m_span[idx_prev_bound] * prev_phase_red_bar + q_span[idx_prev_bound];
    const auto part_c = m_span[idx_next_bound] * phase_diff;
    const auto part_d =
        m_span[idx_next_bound] * phase_red_bar + q_span[idx_next_bound];

    // Broadcast precomputed float part into complex SIMD batchs
    const auto a_batch = CpxBatch::broadcast(part_a);
    const auto b_batch = CpxBatch::broadcast(part_b);
    const auto c_batch = CpxBatch::broadcast(part_c);
    const auto d_batch = CpxBatch::broadcast(part_d);

    // compute the parts that fill whole SIMD batchs
    if constexpr (kWholeSize != 0) {
      for (auto i : iter::range<std::size_t>(0, kWholeSize, kBatchSize)) {
        const auto exp_z_batch = xs::load_aligned(&exp_z_array_[i]);
        const auto z_batch     = xs::load_aligned(&z_array_[i]);
        // (part_a + z * part_b)
        CpxBatch tmp_0 = xs::fma(z_batch, b_batch, a_batch);
        // exp_z * (part_a + z * part_b) - part_c
        CpxBatch tmp_1 = xs::fms(exp_z_batch, tmp_0, c_batch);
        // exp_z * (part_a + z * part_b) - part_c - z * part_d;
        CpxBatch dst_batch = xs::fnma(z_batch, d_batch, tmp_1);
        dst_batch.store_aligned(&aligned_array[i]);
      }
    }

    // compute the parts that fill only parts of a SIMD batchs
    if constexpr (kPartialSize > kBatchThreshold) {
      const auto exp_z_batch = xs::load_aligned(&exp_z_array_[kWholeSize]);
      const auto z_batch     = xs::load_aligned(&z_array_[kWholeSize]);
      // (part_a + z * part_b)
      CpxBatch tmp_0 = xs::fma(z_batch, b_batch, a_batch);
      // exp_z * (part_a + z * part_b) - part_c
      CpxBatch tmp_1 = xs::fms(exp_z_batch, tmp_0, c_batch);
      // exp_z * (part_a + z * part_b) - part_c - z * part_d;
      CpxBatch dst_batch = xs::fnma(z_batch, d_batch, tmp_1);

      dst_batch.store_aligned(&aligned_array[kWholeSize]);
    } else {
      // std::cout << "computeIO partial standard" << std::endl;
      for (std::size_t i = kWholeSize; i < kNumCoeffs; ++i) {
        aligned_array[i] = exp_z_array_[i] * (part_a + z_array_[i] * part_b) -
                           part_c - z_array_[i] * part_d;
      }
    }
  }

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
  inline void computeISumFwd(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept {
    const auto waveform_len = wavetable_->waveformLen(mipmap_idx);
    const auto born_sup =
        jmax_p_red + waveform_len * static_cast<int>(jmin_red > jmax_p_red);

// Get the spans
#if !defined(ADWT_ENABLE_CXX20) & defined(NDEBUG)
    // gsl::span performs bound-checking on the [] operator
    const auto* mdiff_span =
        wavetable_->mDiff(crt_waveform_, mipmap_idx).data();
    const auto* qdiff_span =
        wavetable_->qDiff(crt_waveform_, mipmap_idx).data();
    const auto* phase_span = wavetable_->phases(mipmap_idx).data();
#else
    auto mdiff_span = wavetable_->mDiff(crt_waveform_, mipmap_idx);
    auto qdiff_span = wavetable_->qDiff(crt_waveform_, mipmap_idx);
    auto phase_span = wavetable_->phases(mipmap_idx);
#endif

    // Compute the array of I_sum
    for (auto i : iter::range(jmin_red, born_sup)) {
      const auto i_red = maths::reduce(i, waveform_len);
      const auto phase_red_bar =
          phase_red + static_cast<float>(i_red > jmax_p_red);

      const auto part_a = (phase_red_bar - phase_span[i_red + 1]) / phase_diff;

      // Broadcast precomputed float part into complex SIMD batch
      const auto a_batch          = CpxBatch::broadcast(part_a);
      const auto phase_diff_batch = CpxBatch::broadcast(phase_diff);
      const auto phase_batch      = CpxBatch::broadcast(phase_span[i_red + 1]);
      const auto m_diff_batch     = CpxBatch::broadcast(mdiff_span[i_red]);
      const auto q_diff_batch     = CpxBatch::broadcast(qdiff_span[i_red]);

      if constexpr (kWholeSize != 0) {
        for (auto order : iter::range<std::size_t>(0, kWholeSize, kBatchSize)) {
          const auto z_batch     = xs::load_aligned(&z_array_[order]);
          const auto i_sum_batch = xs::load_aligned(&aligned_array[order]);

          // (phase_diff + z * phase_span[i_red + 1])
          CpxBatch tmp_0 = xs::fma(z_batch, phase_batch, phase_diff_batch);
          // mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
          CpxBatch tmp_1 = tmp_0 * m_diff_batch;
          // (z * qdiff_span[i_red] +
          //  mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
          CpxBatch tmp_2 = xs::fma(z_batch, q_diff_batch, tmp_1);
          // std::exp(z * part_a)
          CpxBatch tmp_3 = xs::exp(z_batch * a_batch);

          // i_sum + std::exp(z * part_a) * tmp2
          CpxBatch dst = xs::fma(tmp_3, tmp_2, i_sum_batch);
          dst.store_aligned(&aligned_array[order]);
        }
      }

      // compute the parts that fill only parts of a SIMD batchs
      if constexpr (kPartialSize > kBatchThreshold) {
        const auto z_batch     = xs::load_aligned(&z_array_[kWholeSize]);
        const auto i_sum_batch = xs::load_aligned(&aligned_array[kWholeSize]);

        // (phase_diff + z * phase_span[i_red + 1])
        CpxBatch tmp_0 = xs::fma(z_batch, phase_batch, phase_diff_batch);
        // mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
        CpxBatch tmp_1 = tmp_0 * m_diff_batch;
        // (z * qdiff_span[i_red] +
        //  mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
        CpxBatch tmp_2 = xs::fma(z_batch, q_diff_batch, tmp_1);
        // std::exp(z * part_a)
        CpxBatch tmp_3 = xs::exp(z_batch * a_batch);

        // i_sum + std::exp(z * part_a) * tmp2
        CpxBatch dst = xs::fma(tmp_3, tmp_2, i_sum_batch);
        dst.store_aligned(&aligned_array[kWholeSize]);
      } else {
        for (std::size_t order = kWholeSize; order < kNumCoeffs; ++order) {
          aligned_array[order] +=
              std::exp(z_array_[order] * part_a) *
              (z_array_[order] * qdiff_span[i_red] +
               mdiff_span[i_red] *
                   (phase_diff + z_array_[order] * phase_span[i_red + 1]));
        }
      }
    }
  }

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
  inline void computeISumBwd(
      std::array<std::complex<float>, kUpperNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept {
    assert(phase_diff < 0);
    const auto waveform_len = wavetable_->waveformLen(mipmap_idx);
    const auto born_sup =
        jmax_p_red + waveform_len * static_cast<int>(jmin_red > jmax_p_red);
    const auto cycle_offset = jmin != 0 && jmin_red > jmax_p_red ? -1.F : 0.F;

// Get the spans
#if !defined(ADWT_ENABLE_CXX20) & defined(NDEBUG)
    // gsl::span performs bound-checking on the [] operator
    const auto* mdiff_span =
        wavetable_->mDiff(crt_waveform_, mipmap_idx).data();
    const auto* qdiff_span =
        wavetable_->qDiff(crt_waveform_, mipmap_idx).data();
    const auto* phase_span = wavetable_->phases(mipmap_idx).data();
#else
    auto mdiff_span = wavetable_->mDiff(crt_waveform_, mipmap_idx);
    auto qdiff_span = wavetable_->qDiff(crt_waveform_, mipmap_idx);
    auto phase_span = wavetable_->phases(mipmap_idx);
#endif

    // Compute the array of I_sum
    for (auto i : iter::range(jmin_red, born_sup)) {
      const auto i_red = maths::reduce(i, waveform_len);
      const auto phase_red_bar =
          phase_red + cycle_offset + static_cast<float>(i_red > jmax_p_red);

      const auto part_a = (phase_red_bar - phase_span[i_red + 1]) / phase_diff;

      // Broadcast precomputed float part into complex SIMD batch
      const auto a_batch          = CpxBatch::broadcast(part_a);
      const auto phase_diff_batch = CpxBatch::broadcast(phase_diff);
      const auto phase_batch      = CpxBatch::broadcast(phase_span[i_red + 1]);
      const auto m_diff_batch     = CpxBatch::broadcast(mdiff_span[i_red]);
      const auto q_diff_batch     = CpxBatch::broadcast(qdiff_span[i_red]);

      if constexpr (kWholeSize != 0) {
        for (auto order : iter::range<std::size_t>(0, kWholeSize, kBatchSize)) {
          const auto z_batch     = xs::load_aligned(&z_array_[order]);
          const auto i_sum_batch = xs::load_aligned(&aligned_array[order]);

          // (phase_diff + z * phase_span[i_red + 1])
          CpxBatch tmp_0 = xs::fma(z_batch, phase_batch, phase_diff_batch);
          // mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
          CpxBatch tmp_1 = tmp_0 * m_diff_batch;
          // (z * qdiff_span[i_red] +
          //  mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
          CpxBatch tmp_2 = xs::fma(z_batch, q_diff_batch, tmp_1);
          // std::exp(z * part_a)
          CpxBatch tmp_3 = xs::exp(z_batch * a_batch);

          // i_sum - std::exp(z * part_a) * tmp2
          CpxBatch dst = xs::fnma(tmp_3, tmp_2, i_sum_batch);
          dst.store_aligned(&aligned_array[order]);
        }
      }

      if constexpr (kPartialSize > kBatchThreshold) {
        const auto z_batch     = xs::load_aligned(&z_array_[kWholeSize]);
        const auto i_sum_batch = xs::load_aligned(&aligned_array[kWholeSize]);

        // (phase_diff + z * phase_span[i_red + 1])
        CpxBatch tmp_0 = xs::fma(z_batch, phase_batch, phase_diff_batch);
        // mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
        CpxBatch tmp_1 = tmp_0 * m_diff_batch;
        // (z * qdiff_span[i_red] +
        //  mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]))
        CpxBatch tmp_2 = xs::fma(z_batch, q_diff_batch, tmp_1);
        // std::exp(z * part_a)
        CpxBatch tmp_3 = xs::exp(z_batch * a_batch);

        // i_sum - std::exp(z * part_a) * tmp2
        CpxBatch dst = xs::fnma(tmp_3, tmp_2, i_sum_batch);
        dst.store_aligned(&aligned_array[kWholeSize]);
      } else {
        for (std::size_t order = kWholeSize; order < kNumCoeffs; ++order) {
          aligned_array[order] -=
              std::exp(z_array_[order] * part_a) *
              (z_array_[order] * qdiff_span[i_red] +
               mdiff_span[i_red] *
                   (phase_diff + z_array_[order] * phase_span[i_red + 1]));
        }
      }
    }
  }

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
      float phase_red) const noexcept {
    assert(wavetable_ != nullptr);
    assert(phase_diff < 0);
    const auto prev_phase_red_bar = prev_phase_red_;
    const auto phase_red_bar = phase_red + static_cast<float>(phase_red == 0.F);

    // Compute I_0
    computeI0(aligned_array, mipmap_idx, jmax_p_red, jmin_red, phase_diff,
              prev_phase_red_bar, phase_red_bar);

    // Compute I_sum
    computeISumBwd(aligned_array, mipmap_idx, jmin, jmin_red, jmax_p_red,
                   phase_diff, phase_red);
  }

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
      float phase_red) const noexcept {
    assert(wavetable_ != nullptr);
    const auto prev_phase_red_bar =
        prev_phase_red_ + static_cast<int>(prev_phase_red_ == 0.F);

    computeI0(aligned_array, mipmap_idx, jmin_red, jmax_p_red, phase_diff,
              prev_phase_red_bar, phase_red);

    computeISumFwd(aligned_array, mipmap_idx, jmin_red, jmax_p_red, phase_diff,
                   phase_red);
  }

  /**
   * @brief Forward-only processing of a span of values
   */
  void processFwd(Span<const float> phases, Span<float> output) {
    assert(wavetable_ != nullptr);
    assert(phases.size() == output.size());
    for (auto&& [phase, output] : iter::zip(phases, output)) {
      auto phase_diff = phase - prev_phase_;
      if (phase_diff < 0.F) {
        phase_diff += 1.F;
      }
      assert(phase_diff > 0 && phase_diff <= 0.5);

      processSampleFwd(phase, phase_diff, output);
    }
  }

  /**
   * @brief Bidirectionnal processing of an entire span of values
   */
  void processBi(Span<const float> phases, Span<float> output) {
    assert(wavetable_ != nullptr);
    assert(phases.size() == output.size());
    for (auto&& [phase, output] : iter::zip(phases, output)) {
      // Ensure the phase diff is in [-0.5; 0.5]
      auto phase_diff = phase - prev_phase_;
      if (phase_diff < -0.5F) {
        phase_diff += 1.F;
      } else if (phase_diff > 0.5F) {
        phase_diff -= 1.F;
      }

      if (phase_diff >= 0.F) {
        processSampleFwd(phase, phase_diff, output);
      } else {
        processSampleBwd(phase, phase_diff, output);
      }
    }
  }

  /**
   * @brief Forward-only processing of a new output value
   * 
   * @param phase The phase of the new value
   * @param phase_diff The phase diff with the previous value
   * @param output Where to store the result
   */
  inline void processSampleFwd(float phase, float phase_diff,
                               float& output) noexcept {
    assert(phase_diff > 0 && phase_diff <= 0.5);

    // Compute mipmap_idx
    const auto&& [mipmap_idx, mipmap_weight, mipmap_idx_up, mipmap_weight_up] =
        wavetable_->findMipMapIndices(phase_diff);

    auto phase_span         = wavetable_->phases(mipmap_idx);
    const auto phase_red    = maths::reduce(phase, 1.F);
    const auto waveform_len = wavetable_->waveformLen(mipmap_idx);

    // Adjust j_red_ if  changing mipmap table
    // "Optimized version" (need benchmark) => shitty output for step over the octave range
    if (mipmap_idx > prev_mipmap_idx_) {
      // Going up in frequencies
      j_red_ = j_red_ >> (mipmap_idx - prev_mipmap_idx_);
    } else if (mipmap_idx < prev_mipmap_idx_) {
      // Going down in frequencies
      j_red_ = j_red_ << (prev_mipmap_idx_ - mipmap_idx);
      if (prev_phase_diff_ > 0) {
        j_red_ += static_cast<int>(phase_span[j_red_ + 1] < prev_phase_red_);
      } else {
        j_red_ -= static_cast<int>(phase_span[j_red_ - 1] > prev_phase_red_);
      }
    }

    // Check if on the same slope as the previous iteration
    prev_j_red_ = j_red_;
    if (prev_phase_diff_ >= 0) {
      prev_j_red_ = j_red_ + maths::sign(prev_phase_red_ - phase_span[j_red_]);
    }

    // Compute edge indices
    j_red_                = maths::floor(phase_red * waveform_len);
    const auto jmax       = j_red_;
    const auto jmin       = prev_j_red_;
    const auto jmin_red   = maths::reduce(jmin - 1, waveform_len);
    const auto jmax_p_red = maths::reduce(jmax, waveform_len);

    alignas(kAligment) auto i_cpx_array =
        std::array<std::complex<float>, kUpperNumCoeffs>();

    // Compute the I complex sum
    computeFwdI(i_cpx_array, mipmap_idx, jmin_red, jmax_p_red, phase_diff,
                phase_red);

    if (mipmap_weight_up != 0.F) {
      // Needs to crossfade with upper mipmap entry
      const auto jmin_red_up   = jmin_red / 2;
      const auto jmax_p_red_up = jmax_p_red / 2;

      alignas(kAligment) auto i_cpx_array_up =
          std::array<std::complex<float>, kUpperNumCoeffs>();

      // Compute the upper mipmap I complex sum
      computeFwdI(i_cpx_array_up, mipmap_idx_up, jmin_red_up, jmax_p_red_up,
                  phase_diff, phase_red);

      // Crossfade
      if constexpr (kPartialSize > kBatchThreshold) {
        for (auto&& [i_cpx, i_cpx_up] :
             iter::zip(i_cpx_array, i_cpx_array_up)) {
          i_cpx = i_cpx * mipmap_weight + i_cpx_up * mipmap_weight_up;
        }
      } else {
        for (auto i : iter::range(kNumCoeffs)) {
          i_cpx_array[i] = i_cpx_array[i] * mipmap_weight +
                           i_cpx_array_up[i] * mipmap_weight_up;
        }
      }
    }

    // Compute the formula (10) for each order
    auto cpx_y_array = std::array<std::complex<float>, kNumCoeffs>();
    for (auto&& [cpx_y, prev_cpx_y, r, z, z_pow2, exp_z, i_cpx] :
         iter::zip(cpx_y_array, prev_cpx_y_array_, r_array_, z_array_,
                   z_pow2_array_, exp_z_array_, i_cpx_array)) {
      cpx_y = exp_z * prev_cpx_y + 2.F * r * (i_cpx / z_pow2);
    }

    // Write the real part of the sum to the output
    output = std::accumulate(cpx_y_array.begin(), cpx_y_array.end(),
                             std::complex<float>{})
                 .real();

    // END
    prev_phase_red_   = phase_red;
    prev_phase_       = phase;
    prev_cpx_y_array_ = cpx_y_array;
    prev_phase_diff_  = phase_diff;
    prev_mipmap_idx_  = mipmap_idx;
  }

  /**
   * @brief Backward-only processing of a new output value
   * 
   * @param phase The phase of the new value
   * @param phase_diff The phase diff with the previous value
   * @param output Where to store the result
   */
  inline void processSampleBwd(float phase, float phase_diff,
                               float& output) noexcept {
    assert(phase_diff < 0 && phase_diff >= -0.5);

    // Compute mipmap_idx
    const auto&& [mipmap_idx, mipmap_weight, mipmap_idx_up, mipmap_weight_up] =
        wavetable_->findMipMapIndices(std::fabs(phase_diff));

    auto phase_span         = wavetable_->phases(mipmap_idx);
    const auto phase_red    = maths::reduce(phase, 1.F);
    const auto waveform_len = wavetable_->waveformLen(mipmap_idx);

    // Adjust j_red_ if  changing mipmap table
    // "Optimized version" (need benchmark) => shitty output for step over the octave range
    if (mipmap_idx > prev_mipmap_idx_) {
      // Going up in frequencies
      j_red_ = j_red_ >> (mipmap_idx - prev_mipmap_idx_);
    } else if (mipmap_idx < prev_mipmap_idx_) {
      // Going down in frequencies
      j_red_ = j_red_ << (prev_mipmap_idx_ - mipmap_idx);
      if (prev_phase_diff_ > 0) {
        j_red_ += static_cast<int>(phase_span[j_red_ + 1] < prev_phase_red_);
      } else {
        j_red_ -= static_cast<int>(phase_span[j_red_ - 1] > prev_phase_red_);
      }
    }

    // // More correct version for steps over an octave range
    // if (mipmap_idx > prev_mipmap_idx_) {
    //   const auto prev_phase_pos = prev_phase_red_ * waveform_len;
    //   if (prev_phase_diff_ >= 0) {
    //     j_red_ = maths::floor(prev_phase_pos);
    //   } else {
    //     j_red_ = maths::ceil(prev_phase_pos);
    //   }
    // }

    // Check if on the same slope as the previous iteration
    prev_j_red_ = j_red_;
    if (prev_phase_diff_ <= 0) {
      prev_j_red_ = j_red_ + maths::sign(prev_phase_red_ - phase_span[j_red_]);
    }

    // Playback going backward
    j_red_    = maths::ceil(phase_red * waveform_len);
    auto jmax = prev_j_red_;
    auto jmin = j_red_;

    // Compute edge indices
    const auto jmin_red   = maths::reduce(jmin - 1, waveform_len);
    const auto jmax_p_red = maths::reduce(jmax, waveform_len);

    alignas(kAligment) auto i_cpx_array =
        std::array<std::complex<float>, kUpperNumCoeffs>();

    // Compute the I complex sum
    computeBwdI(i_cpx_array, mipmap_idx, jmin, jmin_red, jmax_p_red, phase_diff,
                phase_red);

    if (mipmap_weight_up != 0.F) {
      // Needs to crossfade with upper mipmap entry
      const auto jmin_red_up   = jmin_red / 2;
      const auto jmax_p_red_up = jmax_p_red / 2;

      alignas(kAligment) auto i_cpx_array_up =
          std::array<std::complex<float>, kUpperNumCoeffs>();

      // Compute the upper mipmap I complex sum
      computeBwdI(i_cpx_array_up, mipmap_idx_up, jmin, jmin_red_up,
                  jmax_p_red_up, phase_diff, phase_red);

      // Crossfade
      if constexpr (kPartialSize > kBatchThreshold) {
        for (auto&& [i_cpx, i_cpx_up] :
             iter::zip(i_cpx_array, i_cpx_array_up)) {
          i_cpx = i_cpx * mipmap_weight + i_cpx_up * mipmap_weight_up;
        }
      } else {
        for (auto i : iter::range(kNumCoeffs)) {
          i_cpx_array[i] = i_cpx_array[i] * mipmap_weight +
                           i_cpx_array_up[i] * mipmap_weight_up;
        }
      }
    }

    // Compute the formula (10) for each order
    auto cpx_y_array = std::array<std::complex<float>, kNumCoeffs>();
    for (auto&& [cpx_y, prev_cpx_y, r, z, z_pow2, exp_z, i_cpx] :
         iter::zip(cpx_y_array, prev_cpx_y_array_, r_array_, z_array_,
                   z_pow2_array_, exp_z_array_, i_cpx_array)) {
      cpx_y = exp_z * prev_cpx_y + 2.F * r * (i_cpx / z_pow2);
    }

    // Write the real part of the sum to the output
    output = std::accumulate(cpx_y_array.begin(), cpx_y_array.end(),
                             std::complex<float>{})
                 .real();

    // END
    prev_phase_red_   = phase_red;
    prev_phase_       = phase;
    prev_cpx_y_array_ = cpx_y_array;
    prev_phase_diff_  = phase_diff;
    prev_mipmap_idx_  = mipmap_idx;
  }
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