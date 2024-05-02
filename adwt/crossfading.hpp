/*
* crossfading.hpp
* Created by maxmarsc, 24/03/2024
*/

#pragma once

#include <cassert>
#include <cmath>
#include <tuple>
#include <vector>

#include "cppitertools/enumerate.hpp"
#include "maths.hpp"
#include "span.hpp"

namespace adwt {

using MipMapIndices = std::tuple<int, float, int, float>;

class CrossFader {
 public:
  CrossFader(int num_samples, int waveform_len, float samplerate,
             float init_phase_diff = 0.1F)
      : num_samples_(num_samples),
        mipmap_scale_(computeMipMapScale(waveform_len, samplerate)),
        prev_idx_(findMipMapIndex(init_phase_diff)),
        next_idx_(prev_idx_) {}

  inline void resetPhaseDiff(float new_phase_diff) {
    prev_idx_ = findMipMapIndex(new_phase_diff);
    next_idx_ = prev_idx_;
  }

  [[nodiscard]] int prevIdx() const noexcept { return prev_idx_; }

  [[nodiscard]] MipMapIndices newIndices(float abs_phase_diff) {
    auto new_idx = findMipMapIndex(abs_phase_diff);

    if (samples_left_ != 0) {
      // Already crossfading
      assert(prev_idx_ != next_idx_);

      // Check if octave changed
      // NOTE : if new_idx == prev_idx_ it means we went back to the previous
      // octave, in which case we will finish the current transition, which
      // could lead to bad performances as it could reuse cross-fading again and again
      if (new_idx != next_idx_ && new_idx != prev_idx_)
        updateIndices(new_idx);

      samples_left_ -= 1;

      // Check if we reached the end
      if (samples_left_ == 0)
        prev_idx_ = next_idx_;
    } else {
      // Not crossfading yet
      if (new_idx == next_idx_) {
        // No crossfading necessary
        assert(prev_idx_ == next_idx_);
        return std::make_tuple(new_idx, 1.F, new_idx + 1, 0.F);
      }

      // Start crossfading
      samples_left_ = num_samples_;
      updateIndices(new_idx);
    }

    const auto fading = computeFading();
    if (next_idx_ > prev_idx_) {
      return std::make_tuple(prev_idx_, fading, next_idx_, 1.F - fading);
    }
    return std::make_tuple(next_idx_, 1.F - fading, prev_idx_, fading);
  }

  [[nodiscard]] inline int numMipMapTables() const noexcept {
    return static_cast<int>(mipmap_scale_.size());
  }

 private:
  [[nodiscard]] inline float computeFading() const noexcept {
    return static_cast<float>(samples_left_) / static_cast<float>(num_samples_);
  }

  [[nodiscard]] inline int findMipMapIndex(
      float abs_phase_diff) const noexcept {
    assert(abs_phase_diff > 0.F);
    auto i = 0;
    while (i < static_cast<int>(mipmap_scale_.size()) - 1) {
      if (abs_phase_diff < mipmap_scale_[i])
        break;

      ++i;
    }

    return i;
  }

  inline void updateIndices(int new_idx) noexcept {
    if (new_idx > prev_idx_) {
      prev_idx_ = new_idx - 1;
    } else {
      prev_idx_ = new_idx + 1;
    }
    next_idx_ = new_idx;
  }

  static inline std::vector<float> computeMipMapScale(int waveform_len,
                                                      float samplerate) {
    const auto start = samplerate / static_cast<float>(waveform_len) * 2.F;
    const auto num   = computeNumMipMapTables(waveform_len);
    auto scale       = std::vector<float>(num);

    // Compute the mipmap scale
    for (auto&& [i, val] : iter::enumerate(scale)) {
      val = start * static_cast<float>(std::pow(2.F, i)) / samplerate;
    }

    return scale;
  }

  static inline int computeNumMipMapTables(int waveform_len) {
    return maths::floor(std::log2(static_cast<float>(waveform_len) / 4.F)) + 1;
  }

  int num_samples_;
  std::vector<float> mipmap_scale_;
  int samples_left_{};
  int prev_idx_;
  int next_idx_;
};

}  // namespace adwt