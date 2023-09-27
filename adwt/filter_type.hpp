/*
* filter_type.hpp
* Created by maxmarsc, 25/09/2023
*/

#include <array>
#include <complex>
#include <cstddef>
#include <tuple>

namespace adwt {

enum class FilterType { kType1 };

template <FilterType Ftype>
constexpr auto coeffs();

template <FilterType Ftype>
constexpr std::size_t numCoeffs() {
  return std::get<0>(coeffs<Ftype>()).size();
}

template <FilterType Ftype>
using FilterCoeffs = std::tuple<std::array<float, numCoeffs<Ftype>>,
                                std::array<float, numCoeffs<Ftype>>>;

//==============================================================================
template <>
constexpr auto coeffs<FilterType::kType1>() {
  constexpr auto kCount = 1;
  return std::make_tuple(
      std::array<std::complex<float>, 1>{0. - 1.99929732I},
      std::array<std::complex<float>, 1>{-1.99929732 + 1.99929732I});
}

constexpr auto kTest = numCoeffs<FilterType::kType1>();

}  // namespace adwt