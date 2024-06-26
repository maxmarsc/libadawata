/*
* filter_type.hpp
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

#pragma once

#include <array>
#include <complex>
#include <cppitertools/zip.hpp>
#include <cstddef>
#include <tuple>

namespace adwt {

/**
 * @brief Defines the different filters available for the Oscillator class
 *
 * The order of the filters are only even numbers, to use the fact that complex poles
 * filters of successive orders are complex conjugated.
 *
 * With 256-bits SIMD filters of orders up to 16 can be used with approximately
 * the same computation time.
 * With 128-bits SIMD filters of orders up to 8 can be used with approximately
 * the same computation time.
 *
 * For now only experimental filters are available, derived from the research
 * paper https://www.researchgate.net/publication/362628103_Antiderivative_Antialiasing_for_Arbitrary_Waveform_Generation
 * 
 * Here is an example, using Scipy (Python), on how to add a new filter (here a butteworth filter)
 * 
 * 1. Compute the residues and poles values
 * 
 * ..  code-block:: py
 *    from scipy.signal import (
 *        butter,
 *        residue,
 *        zpk2tf,
 *        )
 *    ang_freq = 2 * np.pi * ctf / samplerate
 *    (z, p, k) = butter(order, ang_freq, output="zpk", analog=True)
 *    (b, a) = zpk2tf(z, p, k)
 *    (r, p, _) = residue(b, a)
 * 
 * 
 * 2. Create a new filter type
 * Add a new value in the @ref FilterType enum
 *
 * 3. Fill the new filter coefficients values
 * Specialize the @ref r() & @ref p() functions with the residude and poles values
 * of you new filter. You should only add half the values returned by the scipy code,
 * only the even-indexed one (0, 2, 4, ...)
 * 
 */
enum class FilterType {
  kType1 = 1,  // Butteworth order 2, cutoff at 0.45*samplerate
  kType2 = 2,  // Chebyshev type 2 order 10, cutoff at 0.61*samplerate

  kType3,  // Butterworth order 4
  kType4,  // Butterworth order 6
  kType5,  // Chebyshev type 2 order 8
};

//==============================================================================
template <FilterType Ftype>
constexpr std::size_t numCoeffs();

template <FilterType Ftype>
using CoeffArray = std::array<std::complex<float>, numCoeffs<Ftype>()>;

/**
 * @brief Function to specialize with the residue coefficients of your filter when creating
 * a new filter
 */
template <FilterType Ftype>
constexpr auto r();

/**
 * @brief Function to specialize with the pole coefficients of your filter when creating
 * a new filter
 */
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
// BT2 order 4
template <>
constexpr auto r<FilterType::kType3>() {
  return std::array<std::complex<float>, 2>{-1.30610392 + 0.54100596I,
                                            1.30610392 - 3.15321379I};
}

template <>
constexpr auto z<FilterType::kType3>() {
  return std::array<std::complex<float>, 2>{-1.08201191 + 2.61220784I,
                                            -2.61220784 + 1.08201191I};
}

// BT2 order 6
template <>
constexpr auto r<FilterType::kType4>() {
  return std::array<std::complex<float>, 3>{-4.30788702 - 8.10390177e-14I,
                                            0.57714742 + 9.99648661e-01I,
                                            3.73073959 - 6.46183052e+00I};
}

template <>
constexpr auto z<FilterType::kType4>() {
  return std::array<std::complex<float>, 3>{-1.99929732 + 1.99929732I,
                                            -0.73179361 + 2.73109093I,
                                            -2.73109093 + 0.73179361I};
}

// CH2 order 8
template <>
constexpr auto r<FilterType::kType5>() {
  return std::array<std::complex<float>, 4>{
      0.5995431 - 1.42710928e-03I, -0.10125199 + 2.87722419e+00I,
      -7.73846024 - 1.56288290e+00I, 7.23293555 - 1.16751094e+01I};
}

template <>
constexpr auto z<FilterType::kType5>() {
  return std::array<std::complex<float>, 4>{
      -0.37870934 + 2.57341531I, -1.23202254 + 2.49225066I,
      -2.30871469 + 2.08510777I, -3.31413228 + 0.89104016I};
}

//==============================================================================
template <FilterType Ftype>
constexpr std::size_t numCoeffs() {
  return r<Ftype>().size();
}

}  // namespace adwt