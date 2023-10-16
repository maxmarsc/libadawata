/*
* ovs_downsampler.hpp
* Created by maxmarsc, 08/10/2023
*/

#pragma once

#include "downsampler_base.hpp"
#include "downsampler_juce.hpp"

namespace ovs {

enum class ResamplingBackend { kJuce };

template <ResamplingBackend B>
struct DownsamplerType {
  using Type = void;
};

template <ResamplingBackend B>
using Downsampler = typename DownsamplerType<B>::Type;

//==============================================================================
template <>
struct DownsamplerType<ResamplingBackend::kJuce> {
  using Type = DownsamplerJUCE;
};

}  // namespace ovs