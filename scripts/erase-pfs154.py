#!/usr/bin/env python3

from progproto import *
import argparse

parser = argparse.ArgumentParser(description='Erases Padauk PFS154 chips')
parser.add_argument('com', help='serial port')
args = parser.parse_args()

with Programmer(args.com) as p:
	try:
		p.erase()
	finally:
		p.finish()
