/*
* adwt.hpp
* Created by maxmarsc, 25/09/2023
*/

#pragma once

#if defined(__CLANGD__) && defined(__aarch64__)
#include "extras/clangd_arm_neon.h"
#endif

#include <iostream>
#include <memory>
#include <numeric>
#include <span>

#include "adwt/direction.hpp"
#include "adwt/filter_type.hpp"
#include "adwt/maths.hpp"
#include "adwt/waveform_data.hpp"
#include "cppitertools/range.hpp"

#include <cppitertools/zip.hpp>
#include <tracy/Tracy.hpp>
#include <xsimd/xsimd.hpp>

namespace xs = xsimd;

namespace adwt {

template <FilterType Ftype>
class Oscillator {
  using CpxBatch = xs::batch<std::complex<float>>;

  /**
   * @brief If the number of complex values to compute modulo the size of a SIMD
   * complex batch is above this threshold, we will use a SIMD batch to compute
   * these complex values. If less or equal, we will use a scalar implementation.
   */
  static constexpr auto kBatchThreshold = 1;

  static constexpr std::size_t computeUpperNumCoeffs() {
    constexpr auto kNumCoeffs = numCoeffs<Ftype>();
    constexpr auto kBatchSize = CpxBatch::size;
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
    // return kBatchSize * maths::floor(kNumCoeffs / kBatchSize);
  }

  static constexpr auto kNumCoeffs      = numCoeffs<Ftype>();
  static constexpr auto kUpperNumCoeffs = computeUpperNumCoeffs();
  static constexpr auto kAligment =
      xs::batch<std::complex<float>>::arch_type::alignment();

 public:
  //============================================================================
  Oscillator()
      : waveform_data_(nullptr),
        r_array_(rArray()),
        z_array_(zArray()),
        z_pow2_array_(zPow2Array()),
        exp_z_array_(expZArray()) {}

  //============================================================================
  [[nodiscard]] int init(
      std::unique_ptr<WaveformData>&& waveform_data,
      std::tuple<float, float> init_state = std::make_tuple(0.F, 0.4F)) {
    assert(waveform_data_ == nullptr);
    if (waveform_data == nullptr)
      return 1;
    waveform_data_ = std::move(waveform_data);
    resetInternals(std::get<0>(init_state), std::get<1>(init_state));
    crt_waveform_ = 0;
    return 0;
  }
  [[nodiscard]] std::unique_ptr<WaveformData> swapWaveforms(
      std::unique_ptr<WaveformData>&& waveform_data,
      std::tuple<float, float> init_state = std::make_tuple(0.F, 0.4F)) {
    assert(waveform_data_ != nullptr);
    auto waveform_data_ptr = std::move(waveform_data);
    if (waveform_data_ptr == nullptr)
      return nullptr;
    waveform_data_ptr.swap(waveform_data_);
    resetInternals(std::get<0>(init_state), std::get<1>(init_state));
    crt_waveform_ = 0;
    return waveform_data_ptr;
  }

  //============================================================================
  template <Direction Dir>
  inline void process(std::span<const float> phases, std::span<float> output) {
    if constexpr (Dir == Direction::kForward) {
      processFwd(phases, output);
    } else {
      processBi(phases, output);
    }
  }

  //============================================================================
  [[nodiscard]] inline int numWaveforms() const noexcept {
    assert(waveform_data_ != nullptr);
    return waveform_data_->numWaveforms();
  }
  [[nodiscard]] inline int crtWaveform() const noexcept {
    assert(waveform_data_ != nullptr);
    return crt_waveform_;
  }
  inline void setWaveform(int waveform_idx) noexcept {
    assert(waveform_data_ != nullptr);
    crt_waveform_ = waveform_idx;
  }
  [[nodiscard]] inline std::tuple<float, float> minMaxPhaseDiffRatio()
      const noexcept {
    assert(waveform_data_ != nullptr);
    return waveform_data_->minMaxPhaseDiffRatio();
  };

 private:
  //============================================================================
  void resetInternals(float init_phase, float init_phase_diff) {
    ZoneScoped;
    assert(waveform_data_ != nullptr);
    prev_cpx_y_array_ = std::array<std::complex<float>, kNumCoeffs>{};
    prev_phase_       = init_phase;
    // prev_phase_red_   = std::fmod(init_phase, 1.F);
    prev_phase_red_  = maths::reduce(init_phase, 1.F);
    prev_phase_diff_ = init_phase_diff;
    prev_mipmap_idx_ = std::get<0>(
        waveform_data_->findMipMapIndexes(std::fabs(init_phase_diff)));
    const auto waveform_len = waveform_data_->waveformLen(prev_mipmap_idx_);

    if (init_phase_diff >= 0) {
      j_red_ = maths::floor(prev_phase_red_ * waveform_len);
    } else {
      j_red_ = maths::ceil(prev_phase_red_ * waveform_len);
    }
  }

  inline void computeI0(
      std::array<std::complex<float>, kNumCoeffs>& aligned_array,
      int mipmap_idx, int idx_prev_bound, int idx_next_bound, float phase_diff,
      float prev_phase_red_bar, float phase_red_bar) const noexcept {
    ZoneScoped;
    // Get the span of m & q
    auto m_span = waveform_data_->m(crt_waveform_, mipmap_idx);
    auto q_span = waveform_data_->q(crt_waveform_, mipmap_idx);

    // Compute the recurrent parts of I_0
    const auto part_a = m_span[idx_prev_bound] * phase_diff;
    const auto part_b =
        m_span[idx_prev_bound] * prev_phase_red_bar + q_span[idx_prev_bound];
    const auto part_c = m_span[idx_next_bound] * phase_diff;
    const auto part_d =
        m_span[idx_next_bound] * phase_red_bar + q_span[idx_next_bound];

    // Compute how much we can use SIMD
    using CpxBatch              = xs::batch<std::complex<float>>;
    constexpr auto kBatchSize   = CpxBatch::size;
    constexpr auto kPartialSize = kNumCoeffs % kBatchSize;
    constexpr auto kWholeSize   = kNumCoeffs - kPartialSize;

    // Broadcast precomputed float part into complex SIMD batchs
    const auto a_batch = CpxBatch::broadcast(part_a);
    const auto b_batch = CpxBatch::broadcast(part_b);
    const auto c_batch = CpxBatch::broadcast(part_c);
    const auto d_batch = CpxBatch::broadcast(part_d);

    // compute the parts that fill whole SIMD batchs
    if constexpr (kWholeSize != 0) {
      for (auto i : iter::range<std::size_t>(0, kWholeSize, kBatchSize)) {
        // std::cout << "computeIO whole SIMD" << std::endl;
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
    if constexpr (kPartialSize > 1) {
      // std::cout << "computeIO partial SIMD" << std::endl;
      alignas(kAligment) std::array<std::complex<float>, kUpperNumCoeffs>
          tmp_dst;

      const auto exp_z_batch = xs::load_aligned(&exp_z_array_[kWholeSize]);
      const auto z_batch     = xs::load_aligned(&z_array_[kWholeSize]);
      // (part_a + z * part_b)
      CpxBatch tmp_0 = xs::fma(z_batch, b_batch, a_batch);
      // exp_z * (part_a + z * part_b) - part_c
      CpxBatch tmp_1 = xs::fms(exp_z_batch, tmp_0, c_batch);
      // exp_z * (part_a + z * part_b) - part_c - z * part_d;
      CpxBatch dst_batch = xs::fnma(z_batch, d_batch, tmp_1);
      dst_batch.store_aligned(tmp_dst.data());
      // volatile std::array<std::complex<float>, kUpperNumCoeffs> _ = tmp_dst;
      std::memcpy(&aligned_array[kWholeSize], tmp_dst.data(),
                  kPartialSize * sizeof(std::complex<float>));
    } else {
      // std::cout << "computeIO partial standard" << std::endl;
      for (std::size_t i = kWholeSize; i < kNumCoeffs; ++i) {
        aligned_array[i] = exp_z_array_[i] * (part_a + z_array_[i] * part_b) -
                           part_c - z_array_[i] * part_d;
      }
    }

    //NOLINTNEXTLINE
    // #pragma GCC ivdep
    // for (std::size_t i = 0; i < kNumCoeffs; ++i) {
    //   aligned_array[i] = exp_z_array_[i] * (part_a + z_array_[i] * part_b) -
    //                      part_c - z_array_[i] * part_d;
    // }
  }

  inline void computeISumFwd(
      std::array<std::complex<float>, kNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept {
    ZoneScoped;
    const auto waveform_len = waveform_data_->waveformLen(mipmap_idx);
    const auto born_sup =
        jmax_p_red + waveform_len * static_cast<int>(jmin_red > jmax_p_red);

    // Get the spans
    auto mdiff_span = waveform_data_->mDiff(crt_waveform_, mipmap_idx);
    auto qdiff_span = waveform_data_->qDiff(crt_waveform_, mipmap_idx);
    auto phase_span = waveform_data_->phases(mipmap_idx);

    // Compute how much we can use SIMD
    using CpxBatch              = xs::batch<std::complex<float>>;
    constexpr auto kBatchSize   = CpxBatch::size;
    constexpr auto kPartialSize = kNumCoeffs % kBatchSize;
    constexpr auto kWholeSize   = kNumCoeffs - kPartialSize;

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
          // std::cout << "computeI_SumFwd whole SIMD" << std::endl;
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
      if constexpr (kPartialSize > 1) {
        // std::cout << "computeI_SumFwd partial SIMD" << std::endl;
        alignas(kAligment) std::array<std::complex<float>, kUpperNumCoeffs>
            tmp_dst;

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
        dst.store_aligned(tmp_dst.data());
        // volatile std::array<std::complex<float>, kUpperNumCoeffs> _ = tmp_dst;
        std::memcpy(&aligned_array[kWholeSize], tmp_dst.data(),
                    kPartialSize * sizeof(std::complex<float>));
      } else {
        // std::cout << "computeI_SumFwd partial standard" << std::endl;
        for (std::size_t order = kWholeSize; order < kNumCoeffs; ++order) {
          aligned_array[order] +=
              std::exp(z_array_[order] * part_a) *
              (z_array_[order] * qdiff_span[i_red] +
               mdiff_span[i_red] *
                   (phase_diff + z_array_[order] * phase_span[i_red + 1]));
        }
      }

      // for (auto&& [i_sum, z] : iter::zip(aligned_array, z_array_)) {
      //   i_sum += std::exp(z * part_a) *
      //            (z * qdiff_span[i_red] +
      //             mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]));
      // }
    }
  }

  inline void computeISumBwd(
      std::array<std::complex<float>, kNumCoeffs>& aligned_array,
      int mipmap_idx, int jmin, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept {
    ZoneScoped;
    assert(phase_diff < 0);
    const auto waveform_len = waveform_data_->waveformLen(mipmap_idx);
    const auto born_sup =
        jmax_p_red + waveform_len * static_cast<int>(jmin_red > jmax_p_red);
    const auto cycle_offset = jmin != 0 && jmin_red > jmax_p_red ? -1.F : 0.F;

    // Get the spans
    auto mdiff_span = waveform_data_->mDiff(crt_waveform_, mipmap_idx);
    auto qdiff_span = waveform_data_->qDiff(crt_waveform_, mipmap_idx);
    auto phase_span = waveform_data_->phases(mipmap_idx);

    // Compute the array of I_sum
    for (auto i : iter::range(jmin_red, born_sup)) {
      const auto i_red = maths::reduce(i, waveform_len);
      const auto phase_red_bar =
          phase_red + cycle_offset + static_cast<float>(i_red > jmax_p_red);

      const auto part_a = (phase_red_bar - phase_span[i_red + 1]) / phase_diff;

//NOLINTNEXTLINE
#pragma GCC ivdep
      for (std::size_t i = 0; i < kNumCoeffs; ++i) {
        aligned_array[i] -=
            std::exp(z_array_[i] * part_a) *
            (z_array_[i] * qdiff_span[i_red] +
             mdiff_span[i_red] *
                 (phase_diff + z_array_[i] * phase_span[i_red + 1]));
      }
    }
  }

  std::array<std::complex<float>, kNumCoeffs> computeBiI(
      int mipmap_idx, int jmin, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept {
    ZoneScoped;
    assert(waveform_data_ != nullptr);
    const auto forward = phase_diff > 0;
    const auto prev_phase_red_bar =
        prev_phase_red_ +
        static_cast<int>(prev_phase_red_ == 0.F) * static_cast<int>(forward);
    const auto phase_red_bar =
        phase_red + static_cast<float>(static_cast<int>(phase_red == 0.F) *
                                       static_cast<int>(!forward));

    const auto idx_prev_bound = forward ? jmin_red : jmax_p_red;
    const auto idx_next_bound = forward ? jmax_p_red : jmin_red;

    // Compute the array of I_0
    alignas(alignof(std::complex<float>)) auto i_array =
        std::array<std::complex<float>, kNumCoeffs>();

    computeI0(i_array, mipmap_idx, idx_prev_bound, idx_next_bound, phase_diff,
              prev_phase_red_bar, phase_red_bar);

    if (forward) {
      computeISumFwd(i_array, mipmap_idx, jmin_red, jmax_p_red, phase_diff,
                     phase_red);
    } else {
      computeISumBwd(i_array, mipmap_idx, jmin, jmin_red, jmax_p_red,
                     phase_diff, phase_red);
    }

    return i_array;
  }

  std::array<std::complex<float>, kNumCoeffs> computeBwdI(
      int mipmap_idx, int jmin, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept {
    ZoneScoped;
    assert(waveform_data_ != nullptr);
    assert(phase_diff < 0);
    const auto prev_phase_red_bar = prev_phase_red_;
    const auto phase_red_bar = phase_red + static_cast<float>(phase_red == 0.F);

    // Creates the aligned array for I_0 and I_sum computation
    alignas(kAligment) auto i_array =
        std::array<std::complex<float>, kNumCoeffs>();

    // Compute I_0
    computeI0(i_array, mipmap_idx, jmax_p_red, jmin_red, phase_diff,
              prev_phase_red_bar, phase_red_bar);

    // Compute I_sum
    computeISumBwd(i_array, mipmap_idx, jmin, jmin_red, jmax_p_red, phase_diff,
                   phase_red);

    return i_array;
  }

  std::array<std::complex<float>, kNumCoeffs> computeFwdI(
      int mipmap_idx, int jmin_red, int jmax_p_red, float phase_diff,
      float phase_red) const noexcept {
    ZoneScoped;
    assert(waveform_data_ != nullptr);
    const auto prev_phase_red_bar =
        prev_phase_red_ + static_cast<int>(prev_phase_red_ == 0.F);

    // Compute the array of I_0
    alignas(kAligment) auto i_array =
        std::array<std::complex<float>, kNumCoeffs>();

    computeI0(i_array, mipmap_idx, jmin_red, jmax_p_red, phase_diff,
              prev_phase_red_bar, phase_red);

    computeISumFwd(i_array, mipmap_idx, jmin_red, jmax_p_red, phase_diff,
                   phase_red);

    return i_array;
  }

  void processFwd(std::span<const float> phases, std::span<float> output) {
    ZoneScoped;
    assert(waveform_data_ != nullptr);
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

  void processBi(std::span<const float> phases, std::span<float> output) {
    ZoneScoped;
    assert(waveform_data_ != nullptr);
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

  inline void processSampleFwd(float phase, float phase_diff,
                               float& output) noexcept {
    ZoneScoped;
    assert(phase_diff > 0 && phase_diff <= 0.5);

    // Compute mipmap_idx
    const auto&& [mipmap_idx, mipmap_weight, mipmap_idx_up, mipmap_weight_up] =
        waveform_data_->findMipMapIndexes(phase_diff);

    auto phase_span         = waveform_data_->phases(mipmap_idx);
    const auto phase_red    = maths::reduce(phase, 1.F);
    const auto waveform_len = waveform_data_->waveformLen(mipmap_idx);

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

    // Compute edge indexes
    j_red_                = maths::floor(phase_red * waveform_len);
    const auto jmax       = j_red_;
    const auto jmin       = prev_j_red_;
    const auto jmin_red   = maths::reduce(jmin - 1, waveform_len);
    const auto jmax_p_red = maths::reduce(jmax, waveform_len);

    // Compute the I complex sum
    auto i_cpx_array =
        computeFwdI(mipmap_idx, jmin_red, jmax_p_red, phase_diff, phase_red);

    if (mipmap_weight_up != 0.F) {
      // Needs to crossfade with upper mipmap entry
      const auto jmin_red_up   = jmin_red / 2;
      const auto jmax_p_red_up = jmax_p_red / 2;

      // Compute the upper mipmap I complex sum
      auto i_cpx_array_up = computeFwdI(mipmap_idx_up, jmin_red_up,
                                        jmax_p_red_up, phase_diff, phase_red);

      // Crossfade
      for (auto&& [i_cpx, i_cpx_up] : iter::zip(i_cpx_array, i_cpx_array_up)) {
        i_cpx = i_cpx * mipmap_weight + i_cpx_up * mipmap_weight_up;
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

  inline void processSampleBwd(float phase, float phase_diff,
                               float& output) noexcept {
    ZoneScoped;
    assert(phase_diff < 0 && phase_diff >= -0.5);

    // Compute mipmap_idx
    const auto&& [mipmap_idx, mipmap_weight, mipmap_idx_up, mipmap_weight_up] =
        waveform_data_->findMipMapIndexes(std::fabs(phase_diff));

    auto phase_span         = waveform_data_->phases(mipmap_idx);
    const auto phase_red    = maths::reduce(phase, 1.F);
    const auto waveform_len = waveform_data_->waveformLen(mipmap_idx);

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

    // Compute edge indexes
    const auto jmin_red   = maths::reduce(jmin - 1, waveform_len);
    const auto jmax_p_red = maths::reduce(jmax, waveform_len);

    // Compute the I complex sum
    auto i_cpx_array = computeBwdI(mipmap_idx, jmin, jmin_red, jmax_p_red,
                                   phase_diff, phase_red);

    if (mipmap_weight_up != 0.F) {
      // Needs to crossfade with upper mipmap entry
      const auto jmin_red_up   = jmin_red / 2;
      const auto jmax_p_red_up = jmax_p_red / 2;

      // Compute the upper mipmap I complex sum
      auto i_cpx_array_up = computeBwdI(mipmap_idx_up, jmin, jmin_red_up,
                                        jmax_p_red_up, phase_diff, phase_red);

      // Crossfade
      for (auto&& [i_cpx, i_cpx_up] : iter::zip(i_cpx_array, i_cpx_array_up)) {
        i_cpx = i_cpx * mipmap_weight + i_cpx_up * mipmap_weight_up;
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
  std::unique_ptr<WaveformData> waveform_data_;
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> r_array_;
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> z_array_;
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> z_pow2_array_;
  alignas(kAligment)
      const std::array<std::complex<float>, kUpperNumCoeffs> exp_z_array_;
  std::array<std::complex<float>, kNumCoeffs> prev_cpx_y_array_{};
  float prev_phase_{};
  float prev_phase_red_{};
  float prev_phase_diff_{};
  int prev_mipmap_idx_{};
  int j_red_{};
  int prev_j_red_{};
  int crt_waveform_{};
};

}  // namespace adwt