# Preset manager script for Arduino Zero Stage Metronome
# Erik K. Nyquist 2023
#
# A CLI tool for sending/reading preset data to/from an Arduino Zero Stage Metronome
# via serial port.
#
# Examples:
#
# Downloading all presets from a metronome on COM14 and saving to 'presets.txt':
#
#     $ python scripts/preset_manager.py save -p COM14 -f presets.txt
#
#
# Sending downloaded presets in 'presets.txt' to a metronome on COM14:
#
#     $ python scripts/preset_manager.py load -p COM14 -f presets.txt
#
#
# Creating a new preset called "My preset", BPM = 123, Beat Count = 4, and sending
# to a metronome on COM14:
#
#     $ python scripts/preset_manager.py addpreset -p COM14 -b 123 -c 4 -n "My preset"

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

def _pack_preset_data(bpm, beat_count):
    return ((bpm - 1) & 0x1ff) | (((beat_count - 1) & 0xf) << 9)

def _pack_preset_line(line):
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
    preset_data = _pack_preset_data(bpm, beat_count)

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
        preset_line = _pack_preset_line(line)
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

def _add_preset(ser, bpm, beat_count, name):
    preset_data = _pack_preset_data(bpm, beat_count)
    preset_line = f"0x{preset_data:X} {name}"
    ser.write(f"addpreset {preset_line}\n".encode('utf-8'))

    resp_line = _read_line(ser)
    if not resp_line.startswith('Added preset '):
        print(f"Unrecognized response from metronome: {resp_line}\n")
        return -1

    print(f"Succesfully loaded preset '{name}' to metronome")
    print("\nRemember to power off the metronome via toggle switch or via CLI 'off' command!\n")

def main():
    parser = argparse.ArgumentParser(description='Reads/writes preset data for '
                                                 'Arduino Zero Stage Metronome via USB serial port',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    subparsers = parser.add_subparsers(required=True, help="Operation type")

    save_parser = subparsers.add_parser('save', help='Download all preset data from connected metronome and '
                                        'save to a file. Use "save --help" to see help for this operation.')
    save_parser.add_argument('-p', '--port', required=True, dest='serialport',
                             help='Name of serial port to download presets from (e.g. COM12 or /dev/ttyX).')
    save_parser.add_argument('-f', '--file', required=True, dest='filename',
                             help='Name of file to write downloaded preset data to.')
    save_parser.set_defaults(which='save')

    load_parser = subparsers.add_parser('load', help='Read preset data from a file and send to connected metronome. '
                                                     'Use "load --help" to see help for this operation.')
    load_parser.add_argument('-p', '--port', required=True, dest='serialport',
                             help='Name of serial port to send preset data to (e.g. COM12 or /dev/ttyX).')
    load_parser.add_argument('-f', '--file', required=True, dest='filename',
                             help='Name of file to read preset data from.')
    load_parser.set_defaults(which='load')

    addpreset_parser = subparsers.add_parser('addpreset', help='Create a new preset from BPM, beat count & name '
                                             'provided in command-line options, and send to connected metronome. '
                                             'Use "addpreset --help" to see help for this operation.')
    addpreset_parser.add_argument('-p', '--port', required=True, dest='serialport',
                                  help='Name of serial port to send preset data to (e.g. COM12 or /dev/ttyX).')
    addpreset_parser.add_argument('-b', '--bpm', required=True, type=int, dest='bpm',
                                  help='BPM value for preset, 1 through 512')
    addpreset_parser.add_argument('-c', '--beatcount', required=True, type=int, dest='beat_count',
                                  help='Beat count value for preset, 1 through 16')
    addpreset_parser.add_argument('-n', '--name', required=True, dest='name', help='Name for preset')
    addpreset_parser.set_defaults(which='addpreset')

    args = parser.parse_args()
    ser = serial.Serial(port=args.serialport, baudrate=115200, timeout=0.5)

    if hasattr(args, 'bpm'):
        if (args.bpm < 1) or (args.bpm > 512):
            print(f"Invalid BPM value ({args.bpm}) provided, must be in the range 1-512")
            return -1

    if hasattr(args, 'beat_count'):
        if (args.beat_count < 1) or (args.beat_count > 16):
            print(f"Invalid beat count value ({args.beat_count}) provided, must be in the range 1-16")
            return -1

    # Make sure we have a metronome on this serial port, and report its version if so
    version = _metronome_version_check(ser)
    if version is None:
        print(f"Could not find a metronome at '{args.serialport}'")
        return -1

    print(f"\nFound '{version}' on {args.serialport}")

    ret = -1
    if args.which == 'save':
        ret = _save_preset_data(ser, args.filename)
    elif args.which == 'load':
        ret = _load_preset_data(ser, args.filename)
    elif args.which == 'addpreset':
        ret = _add_preset(ser, args.bpm, args.beat_count, args.name)
    else:
        raise RuntimeError(f"Invalid operation type {args.which}")

    return ret

if __name__ == "__main__":
    sys.exit(main())
