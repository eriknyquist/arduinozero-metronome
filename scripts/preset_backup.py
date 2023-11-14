# Preset backup script for Arduino Zero Stage Metronome
# Erik K. Nyquist 2023
#
# A CLI tool for saving/loading preset data to/from an Arduino Zero Stage Metronome
# via serial port

import sys
import argparse

import serial


def _read_line(ser):
    """
    Reads characters from the serial port until '\n' or '\r' is seen
    """
    buf = ""
    max_read_len = 256
    seen_non_whitespace = False

    for _ in range(max_read_len):
        char = ser.read(1).decode('utf-8')
        if char in ['\n', '\r']:
            if seen_non_whitespace:
                break

        else:
            buf += char
            seen_non_whitespace = True

    return buf

def _unpack_preset(line):
    fields = line.split(':')
    if len(fields) != 2:
        print(f"Unrecognized preset data: {line}\n")
        return None

    data_fields = fields[1].strip().split()
    if len(data_fields) < 2:
        print(f"Unrecognized preset data: {line}\n")
        return None

    preset_name = ' '.join(data_fields[1:])

    try:
        preset_data = int(data_fields[0], 16)
    except ValueError:
        print(f"Unrecognized preset data: {line}\n")
        return None

    bpm = (preset_data & 0x1ff) + 1
    beat_count = (((preset_data) >> 9) & 0xf) + 1

    return f"{bpm},{beat_count},{preset_name}"

def _pack_preset(line):
    fields = line.split(',')
    if len(fields) != 3:
        print(f"Unrecognized preset data: {line}\n")
        return None

    try:
        bpm = int(fields[0])
    except ValueError:
        print(f"Unrecognized preset data: {line}\n")
        return None

    try:
        beat_count = int(fields[1])
    except ValueError:
        print(f"Unrecognized preset data: {line}\n")
        return None

    if 0 in [bpm, beat_count]:
        print(f"Unrecognized preset data: {line}\n")
        return None

    preset_name = fields[2].strip()
    preset_data = ((bpm - 1) & 0x1ff) | (((beat_count - 1) & 0xf) << 9)

    return f"0x{preset_data:X} {preset_name}"

def _metronome_version_check(ser):
    """
    Sends the 'version' command to the serial port, and returns the reported version
    if it matches expected format, and returns None otherwise
    """
    ser.write('version\n'.encode('utf-8'))
    version_line = _read_line(ser)

    if version_line.startswith('Arduino Zero Stage Metronome '):
        return version_line

    return None

def _load_preset_data(ser, filename):
    """
    Reads preset data from 'filename' and loads it to connected metronome
    """
    with open(filename, 'r') as fh:
        lines = fh.readlines()

    if len(lines) == 0:
        print(f"No presets to load in {filename}\n")
        return -1

    for line in lines:
        preset_line = _pack_preset(line)
        if line is None:
            return -1

        ser.write(f"addpreset {preset_line}\n".encode('utf-8'))
        resp_line = _read_line(ser)
        if not resp_line.startswith('Added preset '):
            print(f"Unrecognized response from metronome: {resp_line}\n")
            return -1

    print(f"Succesfully loaded {len(lines)} new presets to metronome")
    print("\nRemember to power off the metronome via toggle switch or via CLI 'off' command!\n")
    return 0

def _save_preset_data(ser, filename):
    """
    Sends the 'presets' command to show all saved preset data, and saves it to a
    test file at 'filename'
    """
    ser.write('presets\n'.encode('utf-8'))
    count_line = _read_line(ser)
    preset_count_str = count_line.split()[0]

    try:
        preset_count = int(preset_count_str)
    except ValueError:
        print(f"Invalid preset count line: {count_line}\n")
        return -1

    if preset_count == 0:
        print("No presets to download\n")
        return 0

    print(f"Downloading {preset_count} presets")

    lines = []
    for i in range(preset_count):
        lines.append(_unpack_preset(_read_line(ser)))

    with open(filename, 'w') as fh:
        fh.write('\n'.join(lines))

    print(f"{preset_count} preset(s) saved in '{filename}'\n")

    return 0

def main():
    parser = argparse.ArgumentParser(description='Backs up & reloads preset data for Arduino Zero Metronome',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('optype', choices=['save', 'load'], help='a string')
    parser.add_argument('serialport', help='Name of serial port to use (e.g. COM12 or /dev/ttyX)')
    parser.add_argument('filename', help=('Name of file to read saved preset data from (if loading),'
                        ' or name of file to write backed up preset data to (if saving).'))
    args = parser.parse_args()
    ser = serial.Serial(port=args.serialport, baudrate=115200, timeout=0.5)

    # Make sure we have a metronome on this serial port, and report its version if so
    version = _metronome_version_check(ser)
    if version is None:
        print(f"Could not find a metronome at '{args.serialport}'")
        return -1

    print(f"\nFound '{version}' on {args.serialport}")

    ret = -1
    if args.optype == 'save':
        ret = _save_preset_data(ser, args.filename)
    else:
        ret =_load_preset_data(ser, args.filename)

    return ret

if __name__ == "__main__":
    sys.exit(main())
