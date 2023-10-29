/*
* span.hpp
* Created by maxmarsc, 29/10/2023
*/

#pragma once

#if __cplusplus == 201703L
#include <gsl/span>
#elif __cplusplus >= 202002L
#include <span>
#else
#error "Unsupported C++ standard"
#endif

namespace adwt {

#if __cplusplus == 201703L
template <typename T>
using Span = gsl::span<T>;
#elif __cplusplus >= 202002L
template <typename T, size_t E = std::dynamic_extent>
using Span = std::span<T, E>;
#else
#error "Unsupported C++ standard (<17)"
#endif

}  // namespace adwt