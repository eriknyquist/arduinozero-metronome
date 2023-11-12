import sys
import wave
import struct

if len(sys.argv) != 2:
    print("Usage: %s <input.wav>" % sys.argv[0])
    sys.exit(1)

fh = wave.open(sys.argv[1])
data = fh.readframes(-1)

framecount = len(data) / 2
vals = []
while data:
    val = data[0:2]
    vals.append(struct.unpack("h", val))
    data = data[2:]

for v in vals:
    print("%s," % v)

print("samples: %d" % framecount)

