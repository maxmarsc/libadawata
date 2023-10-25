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

constexpr auto kTest  = reduce(1, 2);
constexpr auto kTest2 = reduce(1.5F, 1.F);

// inline constexpr float reducef(float value, float threshold) {
//   if (value >= threshold)
// }

// template <typename T, T Threshold>
// inline constexpr T reduce(T value) {
//   static_assert(std::is_arithmetic_v<T>);
//   static_assert(Threshold > T());
//   assert(std::abs(value) < 2 * Threshold);
//   if (value > Threshold) {
//     return value - Threshold;
//   }
//   if (value < Threshold) {
//     return value + Threshold;
//   }
//   return value;
// }

}  // namespace adwt::maths