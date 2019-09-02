#!/usr/bin/env python3

from array import array
from intelhex import IntelHex16bit
from progproto import *
import argparse

parser = argparse.ArgumentParser(description='Writes hex images to Padauk PFS154 chips')
parser.add_argument('--erase', action='store_true', help='Forces a chip erase. If no specified, only if the flash is occupied currently it will be formatted')
parser.add_argument('--no-verify', action='store_true', help='Skips verification')
parser.add_argument('hex', type=argparse.FileType('r'), help='hex file')
parser.add_argument('com', help='serial port')
args = parser.parse_args()

ih = IntelHex16bit(args.hex)

with Programmer(args.com) as p:
	try:
		p.start_write()
		for start, end in ih.segments():
			# Convert to words
			start = start // 2
			end = end // 2

			print('Writing %04X-%04X... ' % (start, end), end='')
			wordarray = ih.tobinarray(start=start, end=end)
			p.write(start, wordarray)
			print('done')
		if not args.no_verify:
			p.start_read()
			for start, end in ih.segments():
				# Convert to words
				start = start // 2
				end = end // 2

				print('Verifying %04X-%04X... ' % (start, end), end='')
				expected = ih.tobinarray(start=start, end=end)
				read = p.read(start, end - start + 1)
				failed = False
				for i in range(0, len(expected)):
					if (expected[i] and 0x3FFF) != (read[i] and 0x3FFF):
						if not failed:
							print('FAILED')
							failed = True
						print('@ %04X - expected %04X, read %04X' % (start + i, expected[i], read[i]))
				if not failed:
					print('OK')
			
	finally:
		p.finish()
