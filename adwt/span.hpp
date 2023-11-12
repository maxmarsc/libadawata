/*
* span.hpp
* Created by maxmarsc, 29/10/2023
*
* This work is licensed under the MIT License
*
* Copyright (c) 2023 Maxime Coutant
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#pragma once

/**
 * @brief In C++17 mode we use GSL's span implementation. Be aware it performs bound checking
 * with the [] operator. It might be better to use a ptr for critical performances
 * parts.
 * 
 */

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