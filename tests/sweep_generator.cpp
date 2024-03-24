/*
* sweep_generator.cpp
* Created by mcoutant, 28/02/2024
*/

#include <argparse/argparse.hpp>
#include <optional>
#include <variant>

#include <sndfile.hh>

#include "adwt/adwt.hpp"
#include "benchmarks/saw_2048.hpp"

constexpr auto kSamplerate = 44100.F;
constexpr auto kDurationS  = 10.F;
constexpr auto kInitPhase  = 0.99F;

void generateLinearSweepPhase(adwt::Span<float> dst, float start, float end,
                              float sr) {
  const auto f_step = (end - start) / static_cast<float>(dst.size());
  auto freq         = start;
  dst[0]            = 0.F;

  for (auto i : iter::range(static_cast<std::size_t>(1), dst.size())) {
    const auto phase_step = freq / sr;
    dst[i]                = std::fmod(dst[i - 1] + phase_step, 1.F);
    freq += f_step;
  }
}

void generateExpSweepPhase(adwt::Span<float> dst, float start, float end,
                           float sr) {
  const auto log_start = std::log2(start);
  const auto log_end   = std::log2(end);
  const auto log_step  = (log_end - log_start) / static_cast<float>(dst.size());
  dst[0]               = 0.F;
  auto log_freq        = log_start;

  for (auto i : iter::range(static_cast<std::size_t>(1), dst.size())) {
    const auto phase_step = std::pow(2.F, log_freq) / sr;
    dst[i]                = std::fmod(dst[i - 1] + phase_step, 1.F);
    log_freq += log_step;
  }
}

float computeFrequency(float phase_diff, float samplerate) {
  if (phase_diff > 0.5F) {
    phase_diff -= 1.0F;
    if (phase_diff < 0.F) {
      phase_diff = std::abs(phase_diff);
    }
  } else if (phase_diff < -0.5F) {
    phase_diff += 1.0F;
  }
  return phase_diff * static_cast<float>(samplerate);
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
      // throw std::invalid_argument("Unsupported filter type");
      std::abort();
    }
  }
}

int main(int argc, char* argv[]) {
  argparse::ArgumentParser program("sweep_generator");

  program.add_argument("fmin")
      .help("Start frequency of the sweep")
      .scan<'g', float>();
  program.add_argument("fmax")
      .help("End frequency of the sweep")
      .scan<'g', float>();
  program.add_argument("ftype")
      .help("Type of filter (1 or 2)")
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

  const auto fmin        = program.get<float>("fmin");
  const auto fmax        = program.get<float>("fmax");
  const auto output_dir  = program.get<std::string>("output_dir");
  const auto ftype       = parseFtype(program.get<int>("ftype"));
  const auto num_samples = static_cast<int>(kSamplerate * kDurationS);
  if (!ftype.has_value()) {
    std::cerr << program;
    return 1;
  }

  // compute the phase vector
  auto phase_vector = std::vector<float>(num_samples);
  // generateLinearSweepPhase(phase_vector, fmin, fmax, kSamplerate);
  generateExpSweepPhase(phase_vector, fmin, fmax, kSamplerate);

  // compute the waveform points
  auto waveform_data =
      adwt::WavetableData::build(benchmarks::kSawWaveform, 1, kSamplerate);
  if (waveform_data == nullptr) {
    std::cerr << "Failed to build WavetableData object" << std::endl;
    return 2;
  }

  // Create and init the oscillator
  auto oscillator        = createOscillator(ftype.value());
  const auto init_result = std::visit(
      [&](auto&& osc) -> int {
        return osc.init(std::move(waveform_data),
                        std::make_tuple(0.99F, 0.01F));
      },
      oscillator);
  if (init_result) {
    std::cerr << "Failed to init oscillator" << std::endl;
    return 3;
  }

  // Process
  auto output_vec = std::vector<float>(num_samples);
  std::visit([&](auto&& osc) { osc.process(phase_vector, output_vec); },
             oscillator);
  for (auto& fval : output_vec) {
    fval *= 0.5F;
  }

  // Compute frequencies from phases
  auto freq_vector = phase_vector;
  freq_vector[0]   = computeFrequency(phase_vector[0] - kInitPhase,
                                      static_cast<float>(kSamplerate));
  for (auto i = 1; i < num_samples; ++i) {
    freq_vector[i] =
        computeFrequency(phase_vector[i] - phase_vector[i - 1], kSamplerate);
  }

  // Write to files
  auto freq_filepath  = output_dir;
  auto audio_filepath = output_dir;
  freq_filepath.append("/frequencies.wav");
  audio_filepath.append("/audio.wav");
  auto freq_sndfile =
      SndfileHandle(freq_filepath, SFM_WRITE, SF_FORMAT_WAV | SF_FORMAT_FLOAT,
                    1, kSamplerate);
  auto audio_sndfile =
      SndfileHandle(audio_filepath, SFM_WRITE, SF_FORMAT_WAV | SF_FORMAT_FLOAT,
                    1, kSamplerate);
  if (!freq_sndfile || !audio_sndfile) {
    std::cerr << "Couldn't open output files for write" << std::endl;
    return 4;
  }
  auto written = freq_sndfile.writef(freq_vector.data(), num_samples);
  if (written != num_samples) {
    std::cerr << "Failed to write frequencies to file" << std::endl;
    return 5;
  }
  written = audio_sndfile.writef(output_vec.data(), num_samples);
  if (written != num_samples) {
    std::cerr << "Failed to write audio to file" << std::endl;
    return 6;
  }

  return 0;
}

// }  // namespace tests