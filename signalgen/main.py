import numpy as np
import sys
import pyaudio
import matplotlib.pyplot as plt
import pysinewave as ps
import keyboard

SAMPLE_RATE = 44100

if __name__ == "__main__":
    p = pyaudio.PyAudio()
    f = float(sys.argv[1])

    modulate = 0.25 * np.sin(2*np.pi*np.linspace(0, SAMPLE_RATE, int(SAMPLE_RATE)) * (f/2)/SAMPLE_RATE)

    samples = (modulate + 0.75 * np.sin(2*np.pi*np.linspace(0, SAMPLE_RATE, int(SAMPLE_RATE)) * f/SAMPLE_RATE)).astype(np.float32)

    stream = p.open(format=pyaudio.paFloat32,
                    channels=1,
                    rate=int(SAMPLE_RATE),
                    output=True)

    # play. May repeat with different volume values (if done interactively)
    while True:
        stream.write(samples)

