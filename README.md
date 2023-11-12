
# Licensing
This work is licensed under the MIT License.

It make use of three different libraries
- [xsimd](https://github.com/xtensor-stack/xsimd/tree/master) licensed under the [BSD3-clause license](https://github.com/xtensor-stack/xsimd/blob/master/LICENSE)
- [libsamplerate](https://github.com/libsndfile/libsamplerate) licensed under the [BSD2-clause license](https://github.com/libsndfile/libsamplerate/blob/master/COPYING)
- [libsndfile]() (only for tests) licensed under the [LGPL v2.1](https://github.com/libsndfile/libsamplerate/blob/master/COPYING)

The benchmarks are licensed under GPL3, see details [here](tests/benchmarks/README.md)

# Milestones
- [x] Basic SIMD optimization
- [x] Add C++17 support
- [x] Remove Tracy (useless)
- [x] Replace SOXR with libsamplerate (license issues)
- [x] Added CMake install configuration
- [x] Documentation
- [x] Add License
- [ ] README
- [ ] Add Sphinx documentation generation
- [ ] Check BUILD_SHARED_LIBS option
- [ ] Use FetchContent for libsndfile
- [ ] Added pkgconfig cmake configuration ?
- [ ] Make LSR dependency optional
- [ ] Test arm32 implementation
- [ ] Add CI/CD
- [ ] Use Github pages for documentation