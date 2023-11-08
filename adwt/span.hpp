/*
* span.hpp
* Created by maxmarsc, 29/10/2023
*/

#pragma once

/**
 * @brief GCC 10.2.1 for aarch64 defines __cplusplus = 201709 when enabling C++20
 * so we can't use __cplusplus to check if C++20 is enabled
 */
#ifdef ADWT_ENABLE_CXX20
#include <span>
#else
#include <gsl/span>
#endif

namespace adwt {
#ifdef ADWT_ENABLE_CXX20
template <typename T, size_t E = std::dynamic_extent>
using Span = std::span<T, E>;
#else
template <typename T>
using Span = gsl::span<T>;
#endif
}  // namespace adwt