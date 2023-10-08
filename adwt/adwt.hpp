/*
* adwt.hpp
* Created by maxmarsc, 25/09/2023
*/

#pragma once

#include <memory>
#include <numeric>
#include <span>

#include "adwt/direction.hpp"
#include "adwt/filter_type.hpp"
#include "adwt/maths.hpp"
#include "adwt/waveform_data.hpp"
#include "cppitertools/range.hpp"

#include <cppitertools/zip.hpp>

namespace adwt {

template <FilterType Ftype>
class Oscillator {
  static constexpr auto kNumCoeffs = numCoeffs<Ftype>();

 public:
  //============================================================================
  Oscillator()
      : waveform_data_(nullptr),
        r_array_(r<Ftype>()),
        z_array_(z<Ftype>()),
        z_pow2_array_(zPow2<Ftype>()),
        exp_z_array_(zExp<Ftype>()) {}

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
    if (waveform_data == nullptr)
      return nullptr;
    // waveform_data.swap(waveform_data_);
    resetInternals(std::get<0>(init_state), std::get<1>(init_state));
    crt_waveform_ = 0;
    return waveform_data;
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

  std::array<std::complex<float>, kNumCoeffs> computeBiI(int mipmap_idx,
                                                         int jmin, int jmin_red,
                                                         int jmax_p_red,
                                                         float phase_diff,
                                                         float phase_red) {
    assert(waveform_data_ != nullptr);
    const auto waveform_len = waveform_data_->waveformLen(mipmap_idx);
    const auto forward      = phase_diff > 0;
    const auto prev_phase_red_bar =
        prev_phase_red_ +
        static_cast<int>(prev_phase_red_ == 0.F) * static_cast<int>(forward);
    const auto phase_red_bar =
        phase_red + static_cast<float>(static_cast<int>(phase_red == 0.F) *
                                       static_cast<int>(!forward));

    const auto idx_prev_bound = forward ? jmin_red : jmax_p_red;
    const auto idx_next_bound = forward ? jmax_p_red : jmin_red;

    // Get the span of each
    auto m_span     = waveform_data_->m(crt_waveform_, mipmap_idx);
    auto q_span     = waveform_data_->q(crt_waveform_, mipmap_idx);
    auto mdiff_span = waveform_data_->mDiff(crt_waveform_, mipmap_idx);
    auto qdiff_span = waveform_data_->qDiff(crt_waveform_, mipmap_idx);
    auto phase_span = waveform_data_->phases(mipmap_idx);

    // Compute the recurrent parts of I_0
    const auto part_a = m_span[idx_prev_bound] * phase_diff;
    const auto part_b =
        m_span[idx_prev_bound] * prev_phase_red_bar + q_span[idx_prev_bound];
    const auto part_c = m_span[idx_next_bound] * phase_diff;
    const auto part_d =
        m_span[idx_next_bound] * phase_red_bar + q_span[idx_next_bound];

    // Compute the array of I_0
    auto i_array = std::array<std::complex<float>, kNumCoeffs>();
    for (auto&& [i_0, z, exp_z] : iter::zip(i_array, z_array_, exp_z_array_)) {
      i_0 = exp_z * (part_a + z * part_b) - part_c - z * part_d;
    }

    // Setup the computation of I_sum
    const auto born_sup =
        jmax_p_red + waveform_len * static_cast<int>(jmin_red > jmax_p_red);
    const auto cycle_offset =
        !forward && jmin != 0 && jmin_red > jmax_p_red ? -1.F : 0.F;

    const auto sign = static_cast<float>(maths::sign(phase_diff));

    // Compute the array of I_sum
    for (auto i : iter::range(jmin_red, born_sup)) {
      const auto i_red = maths::reduce(i, waveform_len);
      const auto phase_red_bar =
          phase_red + cycle_offset + static_cast<float>(i_red > jmax_p_red);

      const auto part_a = (phase_red_bar - phase_span[i_red + 1]) / phase_diff;

      for (auto&& [i_sum, z] : iter::zip(i_array, z_array_)) {
        // const auto part_b = z * qdiff_span[i_red];
        // const auto part_c =
        //     mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]);
        // const auto mem = std::exp(z * part_a) * (part_b + part_c);
        // auto mem =
        //     std::exp(z * part_a) *
        //     (z * qdiff_span[i_red] +
        //      mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]));
        // i_sum += mem;
        i_sum += std::exp(z * part_a) * sign *
                 (z * qdiff_span[i_red] +
                  mdiff_span[i_red] * (phase_diff + z * phase_span[i_red + 1]));
      }
    }

    return i_array;
  }

  /**
    def compute_I_bi(m, q, m_diff, q_diff, X, jmin, jmin_red, jmax_p_red, beta, expbeta, x_diff, prev_x_red, x_red) -> complex:
    frames = m.shape[0]
    prev_x_red_bar = prev_x_red + (prev_x_red == 0.0) * (x_diff > 0)
    x_red_bar = x_red + (x_red == 0.0) * (x_diff < 0)

    if x_diff > 0:
        idx_prev_bound = jmin_red
        idx_next_bound = jmax_p_red
    else:
        idx_prev_bound = jmax_p_red
        idx_next_bound = jmin_red

    I =  expbeta\
            * (m[idx_prev_bound] * x_diff + beta * (m[idx_prev_bound] * prev_x_red_bar + q[idx_prev_bound]))\
            - m[idx_next_bound] * x_diff\
            - beta * (m[idx_next_bound] * x_red_bar + q[idx_next_bound])


    I_sum = 0
    born_sup = jmax_p_red + frames * (jmin_red > jmax_p_red)
    if x_diff < 0 and jmin != 0 and jmin_red > jmax_p_red:
        cycle_offset = -1.0
    else:
        cycle_offset = 0.0
    for i in range(jmin_red, born_sup):
        i_red = i % frames
        x_red_bar = x_red + cycle_offset + (i_red > jmax_p_red)

        I_sum += cexp(beta * (x_red_bar - X[i_red + 1])/x_diff)\
                    * (beta * q_diff[i_red] + m_diff[i_red] * (x_diff + beta * X[i_red + 1]))
        
    # return (I + np.sign(x_diff) * I_sum) / (beta**2)
    return I + I_sum
  */

  void processFwd(std::span<const float> phases, std::span<float> output);
  void processBi(std::span<const float> phases, std::span<float> output) {
    assert(waveform_data_ != nullptr);
    assert(phases.size() == output.size());
    for (auto&& [phase, output] : iter::zip(phases, output)) {
      // Ensure the phase diff is in [-0.5; 0.5]
      auto phase_diff = phase - prev_phase_;
      if (phase_diff < -0.5) {
        phase_diff += 1.F;
      } else if (phase_diff > 0.5F) {
        phase_diff -= 1.F;
      }

      // Compute mipmap_idx
      const auto&& [mipmap_idx, mipmap_weight, mipmap_idx_up,
                    mipmap_weight_up] =
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
      if ((phase_diff >= 0 && prev_phase_diff_ >= 0) ||
          (phase_diff <= 0 && prev_phase_diff_ <= 0)) {
        prev_j_red_ =
            j_red_ + maths::sign(prev_phase_red_ - phase_span[j_red_]);
      }

      // const auto waveform_len = waveform_data_->waveformLen(mipmap_idx);
      auto jmax = 0;
      auto jmin = 0;
      if (phase_diff >= 0) {
        // Playback going forward
        j_red_ = maths::floor(phase_red * waveform_len);
        jmax   = j_red_;
        jmin   = prev_j_red_;
      } else {
        // Playback going backward
        j_red_ = maths::ceil(phase_red * waveform_len);
        jmax   = prev_j_red_;
        jmin   = j_red_;
      }

      // Compute edge indexes
      const auto jmin_red   = maths::reduce(jmin - 1, waveform_len);
      const auto jmax_p_red = maths::reduce(jmax, waveform_len);

      // Compute the I complex sum
      auto i_cpx_array = computeBiI(mipmap_idx, jmin, jmin_red, jmax_p_red,
                                    phase_diff, phase_red);

      if (mipmap_weight_up != 0.F) {
        // Needs to crossfade with upper mipmap entry
        const auto jmin_red_up   = jmin_red / 2;
        const auto jmax_p_red_up = jmax_p_red / 2;

        // Compute the upper mipmap I complex sum
        auto i_cpx_array_up = computeBiI(mipmap_idx_up, jmin, jmin_red_up,
                                         jmax_p_red_up, phase_diff, phase_red);

        // Crossfade
        for (auto&& [i_cpx, i_cpx_up] :
             iter::zip(i_cpx_array, i_cpx_array_up)) {
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
  }

  std::unique_ptr<WaveformData> waveform_data_;
  const std::array<std::complex<float>, kNumCoeffs> r_array_;
  const std::array<std::complex<float>, kNumCoeffs> z_array_;
  const std::array<std::complex<float>, kNumCoeffs> z_pow2_array_;
  const std::array<std::complex<float>, kNumCoeffs> exp_z_array_;
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