/*
* tones_generators.cpp
* Created by maxmarsc, 24/03/2024
*/

#include <argparse/argparse.hpp>
#include <optional>
#include <variant>
#include <vector>

#include <sndfile.hh>

#include "adwt/adwt.hpp"
#include "benchmarks/saw_2048.hpp"
#include "cppitertools/enumerate.hpp"

constexpr auto kSamplerate = 44100.F;
constexpr auto kDurationS  = 1.F;

std::vector<float> generateTonePhase(float freq, int num_points) {
  auto phases           = std::vector<float>(num_points);
  auto crt_phase        = 0.F;
  const auto phase_step = freq / kSamplerate;

  for (auto& phase : phases) {
    crt_phase = std::fmod(crt_phase + phase_step, 1.0F);
    phase     = crt_phase;
  }

  return phases;
}

std::vector<float> generateLogFreqRange(float fmin, float fmax, int fnum) {
  auto freqs = std::vector<float>(fnum);

  const auto log_start = std::log2(fmin);
  const auto log_end   = std::log2(fmax);
  const auto log_step  = (log_end - log_start) / static_cast<float>(fnum);

  auto crt_log_freq = log_start;
  for (auto& freq : freqs) {
    freq = std::pow(2.F, crt_log_freq);
    crt_log_freq += log_step;
  }

  return freqs;
}

std::optional<adwt::FilterType> parseFtype(int int_val) {
  switch (int_val) {
    case 1: {
      return adwt::FilterType::kType1;
    }
    case 2: {
      return adwt::FilterType::kType2;
    }
    default:
      return std::nullopt;
  }
}

using OscillatorType = std::variant<adwt::Oscillator<adwt::FilterType::kType1>,
                                    adwt::Oscillator<adwt::FilterType::kType2>>;

OscillatorType createOscillator(adwt::FilterType ftype) {
  switch (ftype) {
    case (adwt::FilterType::kType1): {
      return adwt::Oscillator<adwt::FilterType::kType1>();
    }
    case (adwt::FilterType::kType2): {
      return adwt::Oscillator<adwt::FilterType::kType2>();
    }
    default: {
      std::abort();
    }
  }
}

std::string formatFreq(float freq) {
  auto str = std::to_string(freq);
  if (freq < 0.F) {
    str.replace(0, 1, "n");
  }
  return str;
}

int generateTone(OscillatorType& osc, float freq, int num_samples,
                 std::string_view output_dir) {
  // Create the input & output vectors
  auto phase_vec  = generateTonePhase(freq, num_samples);
  auto output_vec = std::vector<float>(num_samples);

  // Reset the oscillator phase and phase diff
  const auto start_phase_dif = freq / kSamplerate;
  std::visit(
      [&](auto&& osc) -> void { osc.resetInternals(0.F, start_phase_dif); },
      osc);

  // Compute
  std::visit([&](auto&& osc) { osc.process(phase_vec, output_vec); }, osc);

  // Apply 0.5F gain
  for (auto& output_sample : output_vec) {
    output_sample *= 0.5F;
  }

  // Write to file
  auto output_filepath = std::string(output_dir);
  output_filepath.append("/audio_").append(formatFreq(freq)).append("Hz.wav");
  auto output_file =
      SndfileHandle(output_filepath, SFM_WRITE, SF_FORMAT_WAV | SF_FORMAT_FLOAT,
                    1, kSamplerate);
  if (!output_file) {
    return 1;
  }
  if (output_file.writef(output_vec.data(), num_samples) != num_samples) {
    return 2;
  }
  return 0;
}

int main(int argc, char* argv[]) {
  // Prepare argparse
  argparse::ArgumentParser program("mipmap_transition_generator");

  program.add_argument("ftype")
      .help("Type of filter (1 or 2)")
      .scan<'i', int>();
  program.add_argument("fmin")
      .help("Start frequency of the sweep")
      .scan<'g', float>();
  program.add_argument("fmax")
      .help("End frequency of the sweep")
      .scan<'g', float>();
  program.add_argument("fnum")
      .help("Number of frequency points")
      .scan<'i', int>();
  program.add_argument("output_dir")
      .help("absolute path directory where to store results");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    return 1;
  }

  // Get the typed CLI arguments
  const auto fmin        = program.get<float>("fmin");
  const auto fmax        = program.get<float>("fmax");
  const auto fnum        = program.get<int>("fnum");
  const auto output_dir  = program.get<std::string>("output_dir");
  const auto ftype       = parseFtype(program.get<int>("ftype"));
  const auto num_samples = static_cast<int>(kSamplerate * kDurationS);
  if (!ftype.has_value()) {
    std::cerr << program;
    return 1;
  }

  // compute the waveform points
  auto waveform_data =
      adwt::WavetableData::build(benchmarks::kSawWaveform, 1, kSamplerate);
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    return 2;
  }

  // Create the oscillator
  auto oscillator = createOscillator(ftype.value());

  // Init the oscillator
  const auto init_result = std::visit(
      [&](auto&& osc) -> int { return osc.init(std::move(waveform_data)); },
      oscillator);
  if (init_result) {
    std::cerr << "Failed to init oscillator" << std::endl;
    return 3;
  }

  // Generate the freq range
  auto freqs = generateLogFreqRange(fmin, fmax, fnum);
  for (const auto freq : freqs) {
    for (const auto multiplier : {-1.F, 1.F}) {
      int res =
          generateTone(oscillator, freq * multiplier, num_samples, output_dir);
      switch (res) {
        case 1: {
          std::cerr << "Failed to open output file for writing" << std::endl;
          return 4;
        }
        case 2: {
          std::cerr << "Failed to write to output file" << std::endl;
          return 5;
        }
        default:
          continue;
      }
    }
  }

  return 0;
}