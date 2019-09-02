#!/usr/bin/env python3

from array import array
from intelhex import IntelHex16bit
from progproto import *
import argparse

parser = argparse.ArgumentParser(description='Reads hex images from Padauk PFS154 chips')
parser.add_argument('--start', type=lambda x: int(x, 0), default='0x0000', help='Read start address')
parser.add_argument('--end', type=lambda x: int(x, 0), default='0x07DF', help='Read end address')
parser.add_argument('com', help='serial port')
parser.add_argument('hex', type=argparse.FileType('w'), help='hex file')
args = parser.parse_args()

ih = IntelHex16bit()
ih.padding = 0x3FFF

length = args.end - args.start + 1
if args.start < 0 or args.end > 0x7FF or length <= 0:
	print('Invalid offsets', file=sys.stderr)
	sys.exit(1)

with Programmer(args.com) as p:
	try:
		p.start_read()
		read = p.read(args.start, length)
		ih.puts(args.start * 2, read.tobytes())
		ih.write_hex_file(args.hex)
		with open('raw.bin', 'wb') as f:
			f.write(read.tobytes())
	finally:
		p.finish()
