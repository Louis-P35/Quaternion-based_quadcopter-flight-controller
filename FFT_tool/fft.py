import numpy as np
import matplotlib.pyplot as plt

# Constants
INPUT_FILE = 'signal.txt'
OUTPUT_IMAGE = 'fft_output.png'
SAMPLE_RATE = 1000  # Hz

# Read and parse data with ',' as decimal separator
def load_signal(filename):
    with open(filename, 'r') as f:
        data = [float(line.strip().replace(',', '.')) for line in f if line.strip()]
    return np.array(data)

# Compute FFT and plot
def plot_fft(signal, sample_rate):
    n = len(signal)
    freqs = np.fft.rfftfreq(n, d=1/sample_rate)
    fft_vals = np.abs(np.fft.rfft(signal))

    plt.figure(figsize=(12, 6))
    plt.plot(freqs, fft_vals, color='navy')
    plt.title('FFT of Signal')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(OUTPUT_IMAGE, dpi=300)
    plt.show()

if __name__ == '__main__':
    signal = load_signal(INPUT_FILE)
    plot_fft(signal, SAMPLE_RATE)
