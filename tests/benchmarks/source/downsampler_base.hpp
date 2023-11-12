/*
* ovs_downsampler_base.hpp
* Created by maxmarsc, 11/10/2023
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

#pragma once

#include <cassert>
// #include <span>
#include "adwt/span.hpp"

namespace ovs {

template <typename Derived>
class DownsamplerBase;

template <typename Derived>
struct DownsamplerCtorArgs {};

template <typename Derived>
class DownsamplerBase {
 public:
  DownsamplerBase() = default;

  [[nodiscard]] inline int init(int block_size, float samplerate, int ratio) {
    ratio_ = ratio;
    return static_cast<Derived*>(this)->initDerived(block_size, samplerate,
                                                    ratio);
  }

  inline void process(adwt::Span<float> src, adwt::Span<float> dst) {
    assert(src.size() == dst.size() * ratio_ && !src.empty());
    static_cast<Derived*>(this)->processDerived(src, dst);
  }

  inline void cleanup() { static_cast<Derived*>(this)->cleanupDerived(); }

 protected:
  [[nodiscard]] int initDerived(int block_size, float samplerate, int ratio);
  void processDerived(adwt::Span<float> src, adwt::Span<float> dst);
  void cleanupDerived() {}

  int ratio_{};
};

}  // namespace ovs