
from progproto import *
import serial
import time

with Programmer('COM3') as p:
	try:
		p.start_read()
		print(p.read(0, 64).tobytes().hex())
	finally:
		p.finish()
