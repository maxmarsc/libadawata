/*
* ovs_downsampler_base.hpp
* Created by maxmarsc, 11/10/2023
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