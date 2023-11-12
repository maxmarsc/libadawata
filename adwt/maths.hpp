/*
* maths.hpp
* Created by maxmarsc, 25/09/2023
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

#include <cassert>
#include <cmath>
#include <type_traits>

namespace adwt::maths {

inline constexpr int floor(float fvalue) noexcept {
  const int ivalue = static_cast<int>(fvalue);
  return fvalue >= static_cast<float>(ivalue) ? ivalue : ivalue - 1;
}

inline constexpr int ceil(float fvalue) noexcept {
  const int ivalue = static_cast<int>(fvalue);
  return fvalue > static_cast<float>(ivalue) ? ivalue + 1 : ivalue;
}

inline constexpr bool isOdd(int value) noexcept {
  return (value % 2) == 1;
}

inline constexpr bool isPowerOfTwo(int value) noexcept {
  if (value == 0)
    return false;

  const auto fvalue = static_cast<float>(value);
  return ceil(std::log2f(fvalue)) == floor(std::log2f(fvalue));
}

inline int sign(float value) noexcept {
  if (value == 0.F)
    return 0;
  return std::signbit(value) ? -1 : 1;
}

template <typename T>
inline constexpr T reduce(T value, T threshold) noexcept {
  static_assert(std::is_arithmetic_v<T>);
  assert(threshold > 0);
  assert(value <= 2 * threshold && value >= -threshold);
  if (value >= threshold) {
    return value - threshold;
  }
  if (value < T()) {
    return value + threshold;
  }
  return value;
}

}  // namespace adwt::maths