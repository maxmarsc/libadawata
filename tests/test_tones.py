#! /usr/bin/env python3

from typing import Dict, List, Tuple
import soundfile as sf
import subprocess
import tempfile
from dataclasses import dataclass

import audioaliasingmetrics as aam
import numpy as np
import os
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt
import re

SAMPLERATE = 44100
FMIN = 50
FMAX = 22000
FNUM = 200
BINARY_DIR = Path(os.getenv("BUILD_DIR", Path(__file__).parent.parent / "build"))
TONES_BINARY = BINARY_DIR / "tests" / "tones_generator"

matplotlib.use("TkAgg")


@dataclass
class MetricRange:
    name: str
    snr: np.ndarray[float]
    freqs: np.ndarray[float]


def freq_and_audio_from(audio_file: Path) -> Tuple[float, np.ndarray[np.float32]]:
    PATTERN = r"audio_(n?[-]?\d*\.?\d+)Hz\.wav"
    match = re.search(PATTERN, str(audio_file))
    assert match

    # convert freq to float value
    str_freq = match.group(1)
    if str_freq[0] == "n":
        freq = -1.0 * float(str_freq[1:])
    else:
        freq = float(str_freq)

    data, sr = sf.read(audio_file, dtype=np.float32)
    assert sr == SAMPLERATE
    assert len(data.shape) == 1
    return (freq, data)


def normalized_fft(time_signal: np.ndarray, padding: int = 0) -> np.ndarray:
    signal_len = time_signal.shape[0]

    # Pad with zeros
    padding_len = signal_len + padding * 2
    if padding != 0:
        padded_signal = np.zeros(padding_len)
        padded_signal[padding:-padding] = time_signal
        time_signal = padded_signal

    # window = np.blackman(padding_len)
    window = np.kaiser(padding_len, 38)
    fft = np.fft.rfft(time_signal * window)
    return fft / np.max(np.abs(fft))


def main():
    # Check for the binary
    if not TONES_BINARY.is_file():
        exit("Error : could not find {}".format(TONES_BINARY))

    filter_types = (1, 2)
    magnitudes_dict: Dict[int, Dict[float, np.ndarray[np.float32]]] = dict()

    # Run the tones generator and compute spectral magnitudes
    print("Computing tones")
    for ftype in filter_types:
        tmp_dir = Path(tempfile.mkdtemp())
        proc_args = [
            str(TONES_BINARY.absolute()),
            "{}".format(ftype),
            "{}".format(FMIN),
            "{}".format(FMAX),
            "{}".format(FNUM),
            str(tmp_dir.absolute()),
        ]
        print(tmp_dir)
        completed = subprocess.run(proc_args)
        if completed.returncode != 0:
            exit("sweep generator failed")

        # Sort all generated audio per frequency
        audio_files = sorted(tmp_dir.glob("audio_*.wav"))
        freq_dict: Dict[float, np.ndarray[np.float32]] = dict()
        for audio_file in audio_files:
            freq, audio = freq_and_audio_from(audio_file)
            # Compute the spectral magnitude
            freq_dict[freq] = np.abs(normalized_fft(audio))

        magnitudes_dict[ftype] = freq_dict

    metric_list: List[MetricRange] = list()

    # Computes SNR
    print("Computing SNR")
    for ftype in filter_types:
        magnitudes = magnitudes_dict[ftype]
        all_freqs = np.array(list(magnitudes.keys()))
        pos_freqs = np.sort(all_freqs[all_freqs > 0])
        neg_freqs = np.flip(np.sort(all_freqs[all_freqs < 0]))
        pos_freqs_snr = list()
        neg_freqs_snr = list()

        for pos_freq in pos_freqs:
            magnitude = magnitudes[pos_freq]
            snr = aam.snr(magnitude, SAMPLERATE, pos_freq, aam.Harmonics.ALL)
            pos_freqs_snr.append(snr)

        for neg_freq in neg_freqs:
            magnitude = magnitudes[neg_freq]
            snr = aam.snr(magnitude, SAMPLERATE, abs(neg_freq), aam.Harmonics.ALL)
            neg_freqs_snr.append(snr)

        metric_list.append(
            MetricRange("Ftype {} forward".format(ftype), pos_freqs_snr, pos_freqs)
        )
        metric_list.append(
            MetricRange("Ftype {} backward".format(ftype), neg_freqs_snr, pos_freqs)
        )

    for metric_data in metric_list:
        plt.plot(metric_data.freqs, metric_data.snr, label=metric_data.name)

    plt.title("SNR (Higher is better)")  # Title of the plot
    plt.xlabel("Frequency (Hz)")  # Label for the x-axis
    plt.xscale("log")
    plt.ylabel("SNR (dB)")  # Label for the y-axis
    plt.legend()  # Show the legend to differentiate the two lines
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
