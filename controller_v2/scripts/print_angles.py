import sys
import serial
import struct

if len(sys.argv) > 1:
    port = sys.argv[1]
else:
    port = "/dev/ttyUSB0"

s = serial.Serial(port, 115200)

while True:
    l = int.from_bytes(s.read(1), "little")
    msg = s.read(l)
    op, x, y, z = struct.unpack(">Biii", msg)
    print(f"x: {x/100}, y:{y/100}, z: {z/100}")
