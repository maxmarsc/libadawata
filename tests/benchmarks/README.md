Here are speed benchmarks for the `libadawata` library, and a benchmark of JUCE's
oversampling implementation, to have a comparison.

It would be interesting (important ?) to add more benchmarks :
- [ ] Add other oversampling implementations
- [ ] Add benchmarks of bidirectionnal random values

# Results
These are the benchmarks of the version v0.3.0 presented at ADC23


![v0.3.0 AVX2](.images/v0.3.0_sweep_x86_64_AVX2.png)
![v0.3.0 AVX2 + FMA3](.images/v0.3.0_sweep_x86_64_AVX2_FMA3.png)
![v0.3.0 NEON64](.images/v0.3.0_sweep_arm64_cortex_a53.png)


# Licensing
These benchmarks are licensed under the [GNU General Public License v.3](https://www.gnu.org/licenses/gpl-3.0.en.html)

They make use of the following libraries
- [xsimd](https://github.com/xtensor-stack/xsimd) licensed under the [BSD3-clause license](https://github.com/xtensor-stack/xsimd/blob/master/LICENSE)
- [libsamplerate](https://github.com/libsndfile/libsamplerate) licensed under the [BSD2-clause license](https://github.com/libsndfile/libsamplerate/blob/master/COPYING)
- [libsndfile](https://github.com/libsndfile/libsndfile) (only for tests) licensed under the [LGPL v2.1](https://github.com/libsndfile/libsamplerate/blob/master/COPYING)
- [JUCE](https://github.com/juce-framework/JUCE) licensed under the [GPLv3](https://github.com/juce-framework/JUCE/blob/master/LICENSE.md)
