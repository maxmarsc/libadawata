/*
* utils_benchmarks.hpp
* Created by maxmarsc, 08/10/2023
*/

#pragma once

// #include <span>
#include "adwt/span.hpp"

namespace benchmarks {

void generateLinearSweepPhase(adwt::Span<float> dst, float start, float end,
                              float sr);

}  // namespace benchmarks