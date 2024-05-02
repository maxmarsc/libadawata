#! /usr/bin/env python3

from typing import Dict
import soundfile as sf
import subprocess
import tempfile

# import audioaliasingmetrics as aam
import numpy as np
import os
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt

SAMPLERATE = 44100
FMIN = 50
FMAX = 21500
NFFT = 512
NOVERLAP = int(NFFT * 0.9)
BINARY_DIR = Path(os.getenv("BUILD_DIR", Path(__file__).parent.parent / "build"))
SWEEP_BINARY = BINARY_DIR / "tests" / "sweep_generator"

matplotlib.use("TkAgg")


def plot_specgram(time_signals: Dict[int, np.ndarray[float]]):
    fig, axs = plt.subplots(len(time_signals))

    for i, (ftype, data) in enumerate(time_signals.items()):
        name = "Ftype {}".format(ftype)
        axs[i].specgram(data, NFFT=NFFT, noverlap=NOVERLAP, vmin=-120, Fs=44100)
        axs[i].set_title(name)
        axs[i].set_ylabel("Frequency [Hz]")
        axs[i].set_xlabel("Time [s]")
        axs[i].legend()

    plt.show()


def main():
    # Check for the binary
    if not SWEEP_BINARY.is_file():
        exit("Error : could not find {}".format(SWEEP_BINARY))

    filter_types = (1, 2)
    output_dict = dict()

    # Run the sweep generator
    for ftype in filter_types:
        tmp_dir = Path(tempfile.mkdtemp())
        proc_args = [
            str(SWEEP_BINARY.absolute()),
            "{}".format(ftype),
            "{}".format(FMIN),
            "{}".format(FMAX),
            str(tmp_dir.absolute()),
        ]
        completed = subprocess.run(proc_args)
        if completed.returncode != 0:
            exit("sweep generator failed")
        output_dict[ftype], sr = sf.read(tmp_dir / "audio.wav")
        assert sr == SAMPLERATE
        assert len(output_dict[ftype].shape) == 1

    plot_specgram(output_dict)


if __name__ == "__main__":
    main()
