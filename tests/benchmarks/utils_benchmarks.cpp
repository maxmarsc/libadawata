/*
* utils_benchmarks.cpp
* Created by maxmarsc, 08/10/2023
*/

#include "utils_benchmarks.hpp"
#include <cmath>

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

void generateLinearSweepPhase(std::span<float> dst, float start, float end,
                              float sr) {
  const auto size    = dst.size();
  auto crt_freq      = start;
  auto phase         = 0.1F;
  const auto f_delta = (end - start) / static_cast<float>(size);
  // auto delta = crt_freq / sr;

  for (auto& phase_val : dst) {
    const auto delta = crt_freq / sr;
    phase_val        = std::fmod(phase, 1.F);
    phase += delta;
    crt_freq += f_delta;
  }
}

}  // namespace benchmarks