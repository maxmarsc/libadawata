/*
* utils_benchmarks.cpp
* Created by maxmarsc, 08/10/2023
*
* libadawata benchmarks
* Copyright (C) 2023  Maxime Coutant
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cmath>
#include "cppitertools/range.hpp"
#include "utils_benchmarks.hpp"

namespace benchmarks {

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