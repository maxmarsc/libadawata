/*
* filter_type.hpp
* Created by maxmarsc, 25/09/2023
*/

#include <array>
#include <complex>
#include <cppitertools/zip.hpp>
#include <cstddef>
#include <tuple>

namespace adwt {

enum class FilterType {
  kType1,  // Butteworth order 2, cutoff at 0.45*samplerate
  kType2,  // Chebyshev type 2 order 10, cutoff at 0.61*samplerate
};

//==============================================================================
template <FilterType Ftype>
constexpr std::size_t numCoeffs();

template <FilterType Ftype>
using CoeffArray = std::array<std::complex<float>, numCoeffs<Ftype>()>;

template <FilterType Ftype>
constexpr auto r();

template <FilterType Ftype>
constexpr auto z();

template <FilterType Ftype>
CoeffArray<Ftype> zPow2() {
  constexpr auto kZ = z<Ftype>();
  CoeffArray<Ftype> z_pow2{};

  for (auto&& [z, z_pow2] : iter::zip(kZ, z_pow2)) {
    z_pow2 = z * z;
  }

  return z_pow2;
}

template <FilterType Ftype>
constexpr CoeffArray<Ftype> zExp() {
  constexpr auto kZ = z<Ftype>();
  CoeffArray<Ftype> z_exp{};

  for (auto&& [z, z_exp] : iter::zip(kZ, z_exp)) {
    z_exp = std::exp(z);
  }

  return z_exp;
}

//==============================================================================
template <>
constexpr auto r<FilterType::kType1>() {
  return std::array<std::complex<float>, 1>{0. - 1.99929732I};
}

template <>
constexpr auto z<FilterType::kType1>() {
  return std::array<std::complex<float>, 1>{-1.99929732 + 1.99929732I};
}

//==============================================================================
template <>
constexpr auto r<FilterType::kType2>() {
  return std::array<std::complex<float>, 5>{
      0.26281952 - 0.41698263I, 2.34161433 + 1.04896884I,
      -2.63431397 + 7.51745447I, -16.67718259 - 6.76247888I,
      16.69636533 - 23.51135536I};
}

template <>
constexpr auto z<FilterType::kType2>() {
  return std::array<std::complex<float>, 5>{
      -0.29931827 + 2.94764072I, -0.97441784 + 2.98286211I,
      -1.89003202 + 2.94796957I, -3.15580399 + 2.50801272I,
      -4.37780671 + 1.08149105I};
}

//==============================================================================
template <FilterType Ftype>
constexpr std::size_t numCoeffs() {
  return r<Ftype>().size();
}

constexpr auto kTest  = numCoeffs<FilterType::kType1>();
constexpr auto kTest2 = r<FilterType::kType1>();
constexpr auto kTest3 = z<FilterType::kType1>();
const auto kTest4     = zPow2<FilterType::kType1>();
const auto kTest5     = zExp<FilterType::kType1>();

}  // namespace adwt