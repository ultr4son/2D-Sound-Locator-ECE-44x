import numpy as np
import sys
import pyaudio
import matplotlib.pyplot as plt
import math

SAMPLE_RATE = 44100

if __name__ == "__main__":
    p = pyaudio.PyAudio()
    f = float(sys.argv[1])
    on_time = float(sys.argv[2])

    modulate =  0.25 * np.sin(2*np.pi*np.linspace(0, SAMPLE_RATE * on_time, int(SAMPLE_RATE * on_time)) * (f/4)/SAMPLE_RATE)
    samples = modulate * np.sin(2*np.pi*np.linspace(0, SAMPLE_RATE * on_time, int(SAMPLE_RATE * on_time)) * f/SAMPLE_RATE)
    plt.plot(samples)
    plt.show()
    stream = p.open(format=pyaudio.paFloat32,
                    channels=1,
                    rate=int(SAMPLE_RATE),
                    output=True)

    # play. May repeat with different volume values (if done interactively)
    while True:
        stream.write(samples)

