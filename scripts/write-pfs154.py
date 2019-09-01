#!/usr/bin/env python3

from progproto import *
from array import array
import time

data = array('H', [0] * 8)


with Programmer('COM3') as p:
	try:
		p.start_write()
		for i in range(0, 48, 8):
			print(i)
			p.write(i, data)
			#time.sleep(0.01)
	finally:
		p.finish()
