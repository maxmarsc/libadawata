/*
* utils_benchmarks.hpp
* Created by maxmarsc, 08/10/2023
*/

#pragma once

#include <span>

namespace benchmarks {

void generateLinearSweepPhase(std::span<float> dst, float start, float end,
                              float sr);

};  // namespace benchmarks