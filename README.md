`libadawata` is a C++17/20 library that implements a real-time compatible wavetable
oscillator with Antiderivative Anti-Aliasing based on a Infinite Impulse Response
filter (ADAA-IIR for short).


# Introduction
This is based on the work of Leonardo Gabrielli, Stefano D'Angelo, Pier Paolo La Pastina and Stefano Squartini :  
[*Antiderivative Antialiasing for Arbitrary Waveform Generation*](https://www.researchgate.net/publication/362628103_Antiderivative_Antialiasing_for_Arbitrary_Waveform_Generation) - August 2022

This implementation can provide really strong anti-aliasing, with filters of orders
up to 16, for a complexity similar to an oversampling x8 (see benchmarks).

It was first published at the Audio Developer Conference 2023, you can find the slides [here](https://docs.google.com/presentation/d/1mx8f7yxXMLxQ-pl3IcoqLkcZtQGd7z6gOidcQMAfxPc/edit?usp=sharing)

## What changed since ADC23
- `libadawata` now uses a time-based cross-fading between mipmap tables (~= octaves)
- aarch64 implementations was fixed
- clang-14 support was added
- `ffast-math` is no longer invasive (made PRIVATE)

# Requirements
`libadawata` requires a C++17 or C++20 compliant compiler. It has been tested
with the following setup
| Arch    | Compiler | Config | Status |
| -------- | ------- | ------- | ------- |
| x84_64  | g++-11 | SSE2   | OK
| x84_64  | g++-11 | SSE2 + FMA3 | OK
| x84_64 | g++-10  | SSE2 | OK
| x84_64 | g++-10  | SSE2 + FMA3 | OK
| x84_64 | clang-14 | SSE2 + FMA3 | OK
| aarch64 | g++ 10 | neon64 | OK

It depends on the following libraries :
- [xsimd](https://github.com/xtensor-stack/xsimd/tree/master)
- [libsamplerate](https://github.com/libsndfile/libsamplerate)
- [libsndfile](https://github.com/libsndfile/libsamplerate) (only for tests)

See [Licensing](#Licensing) for licensing details

# How to use
Everything is under the `adwt::` namespace. There are two classes to use :
- `adwt::Oscillator` : the main oscillator class
- `adwt::WavetableData` : the class used to load wavetable into the oscillator

Here is a basic example on how to create an oscillator, and load a waveform :
```cpp
#include <adwt/adwt.hpp>
#include <iostream>
#include <vector>

const auto num_waveforms = 128;
const auto waveform_len = 2048;
auto my_wavetable = std::vector<float>(num_waveforms * waveform_len);
// TODO: Fill the vector with your wavetable

// Load the wavetable
auto wavetable_data = adwt::WavetableData::build(my_wavetable, num_waveforms, 44100);
if (wavetable_data == nullptr) {
    std::cerr << "Failed to load the wavetable" << std::endl;
    std::abort();
}

// Create your oscillator and pick a filter
auto osc = adwt::Oscillator<adwt::FilterType::kType5>{};

// Init your oscillator
if (osc.init(std::move(wavetable_data)) != 0) {
  std::cerr << "Failed to init oscillator" << std::endl;
  std::abort();
}
```

You can then safely generate audio on your audio thread :
```cpp
constexpr auto kBlockSize = 256;
const auto phases_input = std::array<float, kBlockSize>{};
auto audio_output = std::array<float, kBlockSize>{};

osc.process(phases_input, audio_output);
```

In order to change the wavetable loaded in the oscillator, the workflow is :  
1. Asynchronously compute a new wavetable by calling `adwt::WavetableData::build()`
2. Synchronously swap the wavetable using `adwt::Oscillator::swapWavetable()`

You can select the waveform to use in the wavetable with `adwt::Oscillator::setWaveform()`

## Limitations
Due to the nature of the algorithm (see the [slides](https://docs.google.com/presentation/d/1mx8f7yxXMLxQ-pl3IcoqLkcZtQGd7z6gOidcQMAfxPc/edit?usp=sharing)),
it has limitations on how fast the frequency of the signal can vary without an 
exponential increase of the complexity. 

**Note that the cross fading strategy changed since the ADC23 presentation.** `libadawata` now uses
a more classic time-based cross-fading between the different mipmap tables.

When creating a `WavetableData` object, the caller can precise how long the cross-fading
takes (default to 5ms). **As long as you limit variations to no more than a single octave
change every 5ms, no artifacts above -90dB will appear.**

### Setting a new frequency
To avoid such problems when you just want to play at a new arbitrary frequency, 
you can use the `adwt::Oscillator::resetInternals()` method to prepare the recursive
state for your new frequency.

The cross-fading / amplitude envelope to transition from the previous frequency
to the new one is left to the caller.

## Pick a filter
This code uses SIMD optimisation to compute several orders of the IIR filter at the same time.
You can check the [slides](https://docs.google.com/presentation/d/1mx8f7yxXMLxQ-pl3IcoqLkcZtQGd7z6gOidcQMAfxPc/edit?usp=sharing)
for more details, but I suggest using an filter of order 8, which should perform
well on any platform with at least 128bit of SIMD register.

Check [filter_type.hpp](adwt/filter_type.hpp) for more details on which filter
are available, and it should be easy to add support for new filter.

# Build
You can build `libadawata` with cmake >= 3.16

# Testing
Automated testing is really hard for this kind of tasks, as robust and unbiased 
measurements are limited and not always reliable.

`libadawata` is shipped with two sets of tests 

## Unit Tests
These tests only test the basic interface of libadawata. ***They don't check the 
generated audio content.***

You can enable them using `-DBUILD_TESTING=ON`. You can run them with CTest or manually :
```bash
./build/tests/libadawata_tests
```

## Quality Tests
These check provide GUI checks for the user. You can enable them using `-DENABLE_QUALITY_TESTS=ON`

You will also need some additional python modules, listed in the `requirements.txt` file,
which you can install using :
```bash
pip install -r requirements.txt
```

### Sweep test
This will print the spectrogram of a logarithmic sweep test. This is useful for
identifying cross-fading issues.

You can run it with cmake :
```bash
cmake -B build -DBUILD_TESTING=ON -DENABLE_QUALITY_TESTS=ON .
cmake --build build --target sweep_test
```

### SNR test
This will compute the SNR of static tones over a logarithmic range of frequencies.
This is usefull to identify degrading quality issue of the whole algorithm.

You can run it with cmake :
```bash
cmake -B build -DBUILD_TESTING=ON -DENABLE_QUALITY_TESTS=ON .
cmake --build build --target snr_test
```

*I strongly encourage you to run this test on **Release** mode*

# Licensing
This work is licensed under the MIT License.

It depends on the following libraries :
- [xsimd](https://github.com/xtensor-stack/xsimd/tree/master) licensed under the [BSD3-clause license](https://github.com/xtensor-stack/xsimd/blob/master/LICENSE)
- [libsamplerate](https://github.com/libsndfile/libsamplerate) licensed under the [BSD2-clause license](https://github.com/libsndfile/libsamplerate/blob/master/COPYING)
- [libsndfile]() (only for tests) licensed under the [LGPL v2.1](https://github.com/libsndfile/libsamplerate/blob/master/COPYING)

The benchmarks are licensed under GPL3, see details [here](tests/benchmarks/README.md)

# Contributions
Contributions are very welcome. I do plan to improve this library a bit (see milestones)
but keep in mind this is a hobby project.

## Algorithm improvement milestones
- [ ] Experiment with filter design to find the best one for each order
- [x] (planned) Move to a more classic time-based cross-fading

## QoL Milestones
- [x] Basic SIMD optimization
- [x] Add C++17 support
- [x] Added CMake install configuration
- [x] Documentation
- [x] Add License
- [x] READMEs
- [x] Find better unit tests, stop using reference tests from python implementation
- [x] Check more compilers and add to doc
- [ ] Check BUILD_SHARED_LIBS option
- [ ] Use FetchContent for libsndfile
- [ ] Make LSR dependency optional
- [ ] Test arm32 implementation
- [ ] Add pkgconfig cmake configuration ?

## R&D
This code was developed based on the python experimentations I made, which you can
find [here](https://github.com/maxmarsc/ADAA_wavetable/)

## Contact
If you want to discuss about it you can open an issue or you can find me on :
- [Discord](https://discordapp.com/users/Groumpf#2353)
- [Twitter](https://twitter.com/Groumpf_)
- [Mastodon](https://piaille.fr/@groumpf)


