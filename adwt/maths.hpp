/*
* maths.hpp
* Created by maxmarsc, 25/09/2023
*/

namespace adwt::maths {

inline constexpr int floor(float fvalue) {
  const int ivalue = static_cast<int>(fvalue);
  return fvalue >= static_cast<float>(ivalue) ? ivalue : ivalue - 1;
}

inline constexpr int ceil(float fvalue) {
  const int ivalue = static_cast<int>(fvalue);
  return fvalue > static_cast<float>(ivalue) ? ivalue + 1 : ivalue;
}

inline constexpr bool isOdd(int value) {
  return (value % 2) == 1;
}

}  // namespace adwt::maths