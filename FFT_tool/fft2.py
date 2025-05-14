import numpy as np
import matplotlib.pyplot as plt

# Constants
INPUT_FILE = 'signal.txt'
OUTPUT_IMAGE = 'fft_output.png'
SAMPLE_RATE = 1000  # Hz

# Read and parse data with ',' as decimal separator and two signals per line
def load_signals(filename):
    sig1, sig2 = [], []
    with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            try:
                if line.strip():
                    parts = line.strip().replace(',', '.').split(';')
                    if len(parts) == 2:
                        val1 = float(parts[0])
                        val2 = float(parts[1])
                        sig1.append(val1)
                        sig2.append(val2)
            except ValueError:
                continue  # Ignore corrupted lines
    return np.array(sig1), np.array(sig2)

# Compute FFT and plot
def plot_fft(signal, sample_rate, label):
    n = len(signal)
    freqs = np.fft.rfftfreq(n, d=1/sample_rate)
    fft_vals = np.abs(np.fft.rfft(signal))
    plt.plot(freqs, fft_vals, label=label)

if __name__ == '__main__':
    signal1, signal2 = load_signals(INPUT_FILE)

    plt.figure(figsize=(12, 6))
    plot_fft(signal1, SAMPLE_RATE, 'Signal 1')
    plot_fft(signal2, SAMPLE_RATE, 'Signal 2')

    plt.title('FFT of Two Signals')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(OUTPUT_IMAGE, dpi=300)
    plt.show()
