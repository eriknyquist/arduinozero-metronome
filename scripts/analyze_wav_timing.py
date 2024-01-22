import sys
import wave
import struct
import argparse

LOW_TRIGGER = 0.02
HIGH_TRIGGER = 0.1


class BufferedWavReader(object):
    def __init__(self, wav, buffersize=512):
        self.wav = wav
        self.framesize = 2 if self.wav.getnchannels() == 1 else 4
        self.max_bufsize = buffersize
        self.bufsize = buffersize
        self.buf = b''
        self.frameindex = buffersize

    def readframe(self):
        if self.frameindex >= self.bufsize:
            self.buf = self.wav.readframes(self.max_bufsize)
            self.bufsize = len(self.buf) / self.framesize
            self.frameindex = 0

        lower = self.frameindex * self.framesize
        upper = lower + 2
        ret = struct.unpack('<h', self.buf[lower:upper])[0]
        self.frameindex += 1
        return ret


def get_beat_times(wav, threshold=40000000, time_constant=0.05):
    length = wav.getnframes()
    samplerate = wav.getframerate()

    reader = BufferedWavReader(wav)

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
        sample = reader.readframe()

        # mean is the average value of sample
        mean = (1.0 - alpha) * mean + alpha * sample

        # variance is the average value of (sample - mean) ** 2
        variance = (1.0 - alpha) * variance + alpha * (sample - mean) ** 2

        # check if we're loud, and record the time if this changes
        new_is_loud = variance > threshold
        if new_is_loud and (not is_loud):
            result.append(i)

        is_loud = new_is_loud

    return result


def create_output_wav(filename, nframes, sample_rate, beats):
    # Create samples for 1 cycle of a 1KHz square wave
    samples_per_half_cycle = int((sample_rate / 1000.0) / 2.0)
    cycle = ([32767] * samples_per_half_cycle) + ([-32768] * samples_per_half_cycle)
    pulse = cycle * 10

    last_beat_sample = -len(pulse)

    with wave.open(filename, 'wb') as wav:
        wav.setparams((1, 2, sample_rate, nframes, "NONE", "not compressed"))

        for beat_sample in beats:
            delta = (beat_sample - last_beat_sample) - len(pulse)
            if delta <= 0:
                break

            # Fill in space after the last beat
            wav.writeframes(struct.pack(f'<{delta}h', *([0] * delta)))

            # Write out the next beat
            wav.writeframes(struct.pack(f'<{len(pulse)}h', *pulse))

            last_beat_sample = beat_sample


def main():
    parser = argparse.ArgumentParser(description='metronome accuracy tester. Analyzes beats/clicks '
                                     'in a .wav file and prints information about BPM, accuracy, and deviation. '
                                     'Only 16-bit .wav files are supported (mono or stereo, any sample rate).',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('-v', '--volume-trigger', default=40000000, type=int, help='Threshold value for detecting a beat.')
    parser.add_argument('-o', '--output-wav', default=None, help='Name of .wav file to create. Output .wav file will '
                        'contain the same number of samples as the input .wav file, and will have loud pulses at the point '
                        'where each beat was detected. This allows easier visualization of when beats are being detected, '
                        'by (for example) opening both .wav files in Audacity and comparing them.')
    parser.add_argument('-t', '--target-bpm', default=None, type=int, help='Target/expected BPM of the input .wav file. '
                        'If unset, then the average BPM detected in the input .wav file will be used as the target \ '
                        'expected BPM.')
    parser.add_argument('filename', help='.wav file to analyze')
    args = parser.parse_args()

    beats = []

    chans = None
    width = None
    rate = None
    frames = None

    with wave.open(args.filename, 'rb') as wav:
        chans = wav.getnchannels()
        width = wav.getsampwidth()
        rate = wav.getframerate()
        frames = wav.getnframes()

        if (width != 2) or (chans not in [1, 2]):
            print("\nError: only 16-bit .wav files with one channel (mono) or two "
                  "channels (stereo) are supported\n")
            return -1

        target_desc = f"{args.target_bpm:.4f}" if args.target_bpm else "Unset, will use average"
        print(f"\nTarget BPM: {target_desc}")

        print(f"\n{args.filename}: {chans} channels, {width * 8} bit samples, {rate} frames per sec, {frames} frames total")

        beats = get_beat_times(wav, threshold=args.volume_trigger)

    if len(beats) <= 2:
        print(f"\nError: need at least 3 beats ({len(beats)} found)\n")
        return -1

    # throw away the first beat
    beats.pop(0)
    beats_ms = [b / (rate / 1000.0) for b in beats]

    beat_times = []
    beat_times_sum = 0
    for i in range(len(beats_ms) - 1):
        beat_time = beats_ms[i + 1] - beats_ms[i]
        beat_times_sum += beat_time
        beat_times.append(beat_time)

    avg_beat_time = beat_times_sum / len(beat_times)

    if args.target_bpm is None:
        target_desc = "average"
        target_beat_time = avg_beat_time
    else:
        target_desc = "target"
        target_beat_time = 60000.0 / args.target_bpm

    max_deviation = 0
    worst_beat_time = target_beat_time
    worst_beat_number = 0

    for i in range(len(beat_times)):
        beat_time = beat_times[i]
        deviation = beat_time - target_beat_time
        if abs(deviation) > abs(max_deviation):
            max_deviation = deviation
            worst_beat_time = beat_time
            worst_beat_number = i

    max_deviation_percent = abs(max_deviation) / (target_beat_time / 100.0)
    avg_bpm = 60000.0 / avg_beat_time
    worst_beat_bpm = 60000.0 / worst_beat_time

    print("\n\nRESULTS:")
    print(f"\n{len(beats) + 1} beats found")
    print(f"\nAverage beat time: {avg_beat_time:.4f}ms ({avg_bpm:.4f} BPM)")
    print(f"\nWorst beat: {worst_beat_time:.4f}ms ({worst_beat_bpm:.4f} BPM)")
    print(f"Worst beat deviation from {target_desc} beat time: {abs(max_deviation):.4f}ms ({max_deviation_percent:.4f}%)")
    print(f"Worst beat position: {worst_beat_number + 1} of {len(beat_times)}\n")

    if args.output_wav is not None:
        create_output_wav(args.output_wav, frames, rate, beats)

    return 0

if __name__ == "__main__":
    sys.exit(main())
