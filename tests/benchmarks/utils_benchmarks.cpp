/*
* utils_benchmarks.cpp
* Created by maxmarsc, 08/10/2023
*/

#include <cmath>
#include "cppitertools/range.hpp"
#include "utils_benchmarks.hpp"

namespace benchmarks {

/**
//
// A = sine wave amplitude
// fs = sample rate (Hz)
// f0 = initial frequency (Hz)
// f1 = final frequency (Hz)
// T_sweep = duration of sweep (s)
//
phi = 0;                      // phase accumulator
f = f0;                       // initial frequency
delta = 2 * pi * f / Fs;      // phase increment per sample
f_delta = (f1 - f0) / (Fs * T_sweep);
                              // instantaneous frequency increment per sample
for each sample
    output = A * sin(phi);    // output sample value for current sample
    phi += delta;             // increment phase accumulator
    f += f_delta;             // increment instantaneous frequency
    delta = 2 * pi * f / Fs;  // re-calculate phase increment
*/

void generateLinearSweepPhase(adwt::Span<float> dst, float start, float end,
                              float sr) {
  const auto f_step = (end - start) / static_cast<float>(dst.size());
  auto freq         = start;
  dst[0]            = 0.F;

  for (auto i : iter::range(static_cast<std::size_t>(1), dst.size())) {
    const auto phase_step = freq / sr;
    dst[i]                = std::fmod(dst[i - 1] + phase_step, 1.F);
    freq += f_step;
  }
}

}  // namespace benchmarks