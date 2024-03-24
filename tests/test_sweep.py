#! /usr/bin/env python3

from dataclasses import dataclass
from typing import Tuple
import soundfile as sf
import subprocess
import tempfile
import audioaliasingmetrics as aam
import unittest
import numpy as np
import os
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt

SAMPLERATE = 44100
DURATION = 10
NUM_SAMPLES = SAMPLERATE * DURATION
FMIN = 50
FMAX = 5000
CROSSFADE_S = 0.005
BINARY_DIR = Path(os.getenv("BUILD_DIR"))
# assert BINARY_DIR != Path(None)
SWEEP_BINARY = BINARY_DIR / "sweep_generator"

matplotlib.use("TkAgg")


@dataclass
class MetricCurve:
    log_factor: float
    offset: float

    def compute_metric(self, freqs: np.ndarray[float]) -> np.ndarray[float]:
        return self.log_factor * np.log(freqs) + self.offset


FTYPE_1_SNR = MetricCurve(-4.6, 58)
FTYPE_2_SNR = MetricCurve(-4.6, 82)
SINAD = MetricCurve(-4.5, 55)
SNR_TOLERANCE = 1.5
SINAD_TOLERANCE = 1.5


def floor_ceil(fval: float) -> Tuple[int, int]:
    ival = int(fval)
    if ival == fval:
        return (ival, ival)
    if ival > fval:
        return (ival - 1, ival)
    return (ival, ival + 1)


def greater_power_of_two(val: int) -> int:
    i = 0
    while 2**i < val:
        i += 1
    return 2**i


def pad_frame(signal: np.ndarray[np.float32]):
    frame_size = signal.shape[0]
    fft_size = greater_power_of_two(frame_size)
    if fft_size == frame_size:
        return signal

    ret = np.zeros((fft_size,), dtype=np.float32)
    padding = fft_size - frame_size
    padding_left, padding_right = floor_ceil(padding / 2.0)
    ret[padding_left:-padding_right] = signal

    return ret


def compute_spectral_magnitude(
    signal_frame: np.ndarray[np.float32],
) -> np.ndarray[np.complex64]:
    # First apply windowing
    # signal_frame *= np.kaiser(signal_frame.shape[0], 38)
    signal_frame *= np.blackman(signal_frame.shape[0])

    # Pad the frame
    signal_frame = pad_frame(signal_frame)

    # Compute the fft
    fft = np.fft.rfft(signal_frame)
    mag = np.abs(fft)
    return mag / np.max(mag)


class TestSweepGeneration(unittest.TestCase):
    def test_sweep_up_type_1(self):
        # Create the working temp dir
        dir = Path(tempfile.mkdtemp()).absolute()
        print("Work in ", dir)

        # Run the sweep generator
        proc_args = [
            SWEEP_BINARY.absolute(),
            "{}".format(FMIN),
            "{}".format(FMAX),
            "1",
            dir.absolute(),
        ]
        completed = subprocess.run(proc_args)
        assert completed.returncode == 0

        audio_path = dir / "audio.wav"
        frequencies_path = dir / "frequencies.wav"
        audio, sr = sf.read(audio_path)
        freqs, _ = sf.read(frequencies_path)

        num_checks = 2 * int(DURATION / CROSSFADE_S)
        idx_checks = np.int32(np.linspace(0, audio.shape[0], num_checks + 2)[1:-1])
        snr_reference = FTYPE_1_SNR.compute_metric(freqs)

        for idx in idx_checks:
            fundamental = freqs[idx]
            wavelength = int(sr / fundamental)
            side_context = int(wavelength * 2.5)
            if idx < wavelength * 10 or idx + wavelength * 10 > freqs.shape[0]:
                continue

            # audio_slice = audio[idx - wavelength : idx + wavelength]
            audio_slice = audio[idx - side_context : idx + side_context]
            laudio = tuple(audio_slice)
            magnitudes = compute_spectral_magnitude(audio_slice)
            lmag = tuple(magnitudes)
            snr = aam.snr(magnitudes, SAMPLERATE, fundamental, aam.Harmonics.ALL)
            if snr < snr_reference[idx] - SNR_TOLERANCE:
                print(
                    "SNR at {}Hz was {}dB : expected {}dB".format(
                        fundamental, snr, snr_reference[idx]
                    )
                )
                # raise ValueError(
                #     "SNR at {}Hz was {}dB : expected {}dB".format(
                #         fundamental, snr, snr_reference[idx]
                #     )
                # )
            # frame = padFrame(audio_slice)
            # windows = np.kaiser(frame.shape[0], 38)
            # window = np.kaiser()


if __name__ == "__main__":
    unittest.main()
