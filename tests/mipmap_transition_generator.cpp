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
#include "cppitertools/enumerate.hpp"

constexpr auto kSamplerate = 44100.F;
constexpr auto kDurationS  = 0.5F;

std::vector<float> generateTransition(float start_phase_diff,
                                      float end_phase_diff, int num_points) {
  auto phases    = std::vector<float>(num_points);
  auto crt_phase = 0.F;

  auto first_part = adwt::Span<float>(phases).subspan(0, num_points / 2);
  auto second_part =
      adwt::Span<float>(phases).subspan(num_points / 2, num_points / 2);

  for (auto& phase : first_part) {
    crt_phase += start_phase_diff;
    phase = std::fmod(crt_phase, 1.F);
  }

  for (auto& phase : second_part) {
    crt_phase += end_phase_diff;
    phase = std::fmod(crt_phase, 1.F);
  }

  return phases;
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

int main(int argc, char* argv[]) {
  // Prepare argparse
  argparse::ArgumentParser program("mipmap_transition_generator");

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

  // Parse cli args
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
  auto mip_map_table = waveform_data->mipMapTable();

  // Create the oscillator
  auto oscillator = createOscillator(ftype.value());

  // Init the oscillator
  const auto init_result = std::visit(
      [&](auto&& osc) -> int {
        return osc.init(std::move(waveform_data), std::make_tuple(0.F, 0.99F));
      },
      oscillator);
  if (init_result) {
    std::cerr << "Failed to init oscillator" << std::endl;
    return 3;
  }

  // We iterator over each transition point
  for (auto&& [i, transition] : iter::enumerate(mip_map_table)) {
    const auto upper_freq = 1.01F * transition;
    const auto lower_freq = 0.99F * transition;

    // Generates frequencies for the transition
    const auto phases_up =
        generateTransition(lower_freq, upper_freq, num_samples);
    const auto phases_down =
        generateTransition(upper_freq, lower_freq, num_samples);

    // Init for phase up transition
    std::visit([&](auto&& osc) -> void { osc.resetInternals(0.F, lower_freq); },
               oscillator);

    // Process phase up transition
    auto output_up = std::vector<float>(num_samples);
    std::visit([&](auto&& osc) { osc.process(phases_up, output_up); },
               oscillator);
    for (auto& fval : output_up) {
      fval *= 0.5F;
    }

    // Init for phase down transition
    std::visit([&](auto&& osc) -> void { osc.resetInternals(0.F, upper_freq); },
               oscillator);

    // Process phase down transition
    auto output_down = std::vector<float>(num_samples);
    std::visit([&](auto&& osc) { osc.process(phases_down, output_down); },
               oscillator);
    for (auto& fval : output_up) {
      fval *= 0.5F;
    }

    // compute freqs from phase
    auto freqs_up = std::vector(phases_up);
    auto freqs_dn = std::vector(phases_down);
    for (auto&& [up, down] : iter::zip(freqs_up, freqs_dn)) {
      up *= kSamplerate;
      down *= kSamplerate;
    }

    // Write to files
    auto freq_up_filepath  = output_dir;
    auto audio_up_filepath = output_dir;
    freq_up_filepath.append("/frequencies_up_")
        .append(std::to_string(i))
        .append(".wav");
    audio_up_filepath.append("/audio_up_")
        .append(std::to_string(i))
        .append(".wav");
    auto freq_dn_filepath  = output_dir;
    auto audio_dn_filepath = output_dir;
    freq_dn_filepath.append("/frequencies_dn_")
        .append(std::to_string(i))
        .append(".wav");
    audio_dn_filepath.append("/audio_dn_")
        .append(std::to_string(i))
        .append(".wav");

    auto freq_up_sndfile =
        SndfileHandle(freq_up_filepath, SFM_WRITE,
                      SF_FORMAT_WAV | SF_FORMAT_FLOAT, 1, kSamplerate);
    auto freq_dn_sndfile =
        SndfileHandle(freq_dn_filepath, SFM_WRITE,
                      SF_FORMAT_WAV | SF_FORMAT_FLOAT, 1, kSamplerate);
    auto audio_up_sndfile =
        SndfileHandle(audio_up_filepath, SFM_WRITE,
                      SF_FORMAT_WAV | SF_FORMAT_FLOAT, 1, kSamplerate);
    auto audio_dn_sndfile =
        SndfileHandle(audio_dn_filepath, SFM_WRITE,
                      SF_FORMAT_WAV | SF_FORMAT_FLOAT, 1, kSamplerate);
    if (!freq_up_sndfile || !freq_dn_sndfile || !audio_up_sndfile ||
        !audio_dn_sndfile) {
      std::cerr << "Couldn't open output files for write" << std::endl;
      return 4;
    }

    if (freq_up_sndfile.writef(freqs_up.data(), num_samples) != num_samples) {
      std::cerr << "Failed to write frequencies to file" << std::endl;
      return 5;
    }
    if (audio_up_sndfile.writef(output_up.data(), num_samples) != num_samples) {
      std::cerr << "Failed to write audio to file" << std::endl;
      return 6;
    }
    if (freq_dn_sndfile.writef(freqs_dn.data(), num_samples) != num_samples) {
      std::cerr << "Failed to write frequencies to file" << std::endl;
      return 7;
    }
    if (audio_dn_sndfile.writef(output_down.data(), num_samples) !=
        num_samples) {
      std::cerr << "Failed to write audio to file" << std::endl;
      return 8;
    }
  }
}