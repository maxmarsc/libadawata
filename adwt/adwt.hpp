/*
* adwt.hpp
* Created by maxmarsc, 25/09/2023
*/

#pragma once

#include <memory>
#include <span>

#include "adwt/direction.hpp"
#include "adwt/filter_type.hpp"
#include "adwt/waveform_data.hpp"

namespace adwt {

template <FilterType Ftype>
class Oscillator {
 public:
  Oscillator();

  //============================================================================
  [[nodiscard]] int init(std::unique_ptr<WaveformData> waveform_data);
  std::unique_ptr<WaveformData> swapWaveform(
      std::unique_ptr<WaveformData> waveform_data);

  //============================================================================
  template <Direction Dir>
  inline void process(std::span<const float> phases, std::span<float> output) {
    if constexpr (Dir == Direction::kForward) {
      processFwd(phases, output);
    } else {
      processBi(phases, output);
    }
  }

 private:
  void processFwd(std::span<const float> phases, std::span<float> output);
  void processBi(std::span<const float> phases, std::span<float> output);

  std::unique_ptr<WaveformData> waveform_data_;
};

}  // namespace adwt