import sys
import wave
import struct
import argparse

LOW_TRIGGER = 0.02
HIGH_TRIGGER = 0.1

def read_buffer(wavfile, size):
    ret = []
    for i in range(size):
        frames = wavfile.readframes(1)
        if len(frames) != 4:
            break

        sample = struct.unpack('<h', frames[:2])[0]
        ret.append(sample)

    return ret

def buffer_amplitude(buf):
    sample_sum = 0
    normalized = []
    for sample in buf:
        if sample < 0:
            normalized.append(sample * -1)
        else:
            normalized.append(sample)

    n_highest = int(len(buf) / 2)
    for sample in sorted(normalized, reverse=True)[:n_highest]:
        sample_sum += sample

    return sample_sum / 32767 / n_highest

def get_beat_times(wav, threshold=0.02, time_constant=0.05):
    length = wav.getnframes()
    samplerate = wav.getframerate()
    threshold *= (65536.0 * 65536.0)

    # Our result will be a list of (time, is_loud) giving the times when
    # when the audio switches from loud to quiet and back.
    is_loud = False
    result = []

    # The following values track the mean and variance of the signal.
    # When the variance is large, the audio is loud.
    mean = 0.0
    variance = 0.0

    # If alpha is small, mean and variance change slower but are less noisy.
    alpha = 1.0 / (time_constant * float(samplerate))

    for i in range(length):
        sample = struct.unpack('<h', wav.readframes(1)[:2])[0]

        # mean is the average value of sample
        mean = (1.0 - alpha) * mean + alpha * sample

        # variance is the average value of (sample - mean) ** 2
        variance = (1.0 - alpha) * variance + alpha * (sample - mean) ** 2

        # check if we're loud, and record the time if this changes
        new_is_loud = variance > threshold
        if new_is_loud and (not is_loud):
            sample_time = float(i) / (samplerate / 1000.0)
            result.append(sample_time)

        is_loud = new_is_loud

    return result

def main():
    parser = argparse.ArgumentParser(description='metronome accuracy tester. Analyzes beats/clicks '
                                     'in a .wav file and prints information about BPM, accuracy, and deviation. '
                                     'Only 16-bit .wav files are supported (mono or stereo, any sample rate).',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-b', '--block-size', default=32, type=int, help='Number of samples to read and analyze '
                        'amplitude for at once. Smaller block sizes equate to higher precision with respect '
                        'to time.')
    parser.add_argument('-l', '--low-trigger', default=0.02, type=float, help='Low trigger value (0.0 through 1.0). '
                        'When a block with average amplitude at or below the low trigger value is seen, followed '
                        'by a block with average amplitude equal or higher than (low_trigger + high_trigger), '
                        'Then a beat/click is detected.')
    parser.add_argument('-u', '--high-trigger', default=0.1, type=float, help='High trigger value (0.0 through 1.0). '
                        'When a block with average amplitude at or below the low trigger value is seen, followed '
                        'by a block with average amplitude equal or higher than (low_trigger + high_trigger), '
                        'Then a beat/click is detected.')
    parser.add_argument('filename', help='.wav file to analyze')
    args = parser.parse_args()

    beats = []

    with wave.open(args.filename, 'rb') as wav:
        chans = wav.getnchannels()
        width = wav.getsampwidth()
        rate = wav.getframerate()
        frames = wav.getnframes()
        if width != 2:
            print("Error: only 16-bit .wav files are supported")
            return -1

        print(f"{chans} channels, {width * 8} bit samples, {rate} frames per sec, {frames} frames total")
        beats = get_beat_times(wav)

    print(f"{len(beats)} beats found")
    if len(beats) <= 1:
        print("Error: need at least 2 beats")
        return -1

    beat_times = []
    beat_times_sum = 0
    for i in range(len(beats) - 1):
        beat_time = beats[i + 1] - beats[i]
        beat_times_sum += beat_time
        beat_times.append(beat_time)

    avg_beat_time = beat_times_sum / len(beat_times)
    max_deviation = 0
    for beat_time in beat_times:
        deviation = beat_time - avg_beat_time
        if abs(deviation) > abs(max_deviation):
            max_deviation = deviation

    max_deviation_percent = abs(max_deviation) / (avg_beat_time / 100.0)
    avg_bpm = 60000.0 / avg_beat_time

    print(f"Average beat time: {avg_beat_time:.4f}ms ({avg_bpm:.4f} BPM)")
    print(f"Max. deviation from average beat time: {abs(deviation):.4f}ms ({max_deviation_percent:.4f}%)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
