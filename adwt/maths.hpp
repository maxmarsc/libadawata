/*
* maths.hpp
* Created by maxmarsc, 25/09/2023
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