
from progproto import *

with Programmer('COM3') as p:
	try:
		p.erase()
	finally:
		p.finish()
