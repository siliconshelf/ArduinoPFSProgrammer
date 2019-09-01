
from enum import Enum
from cobs import cobs
import array
import struct
import serial

class RequestId(Enum):
	MODE = 0
	READ = 1
	WRITE = 2
	ERASE = 3

class ProtoModes(Enum):
	OFF = 0
	READ = 1
	WRITE = 2
	ERASE = 3

class ReplyId(Enum):
	OK = 0
	DEVICE_ID = 1
	READ = 2

	UNKNOWN_REQUEST = 0x80
	INVALID_REQUEST_LENGTH = 0x81
	INVALID_REQUESTED_MODE = 0x82
	INVALID_CURRENT_MODE = 0x83
	REPLY_CHUNK_TOO_LARGE = 0x84

class Programmer:
	MAX_WORDS = 64

	def __init__(self, port):
		if isinstance(port, serial.Serial):
			self._serial = port
		else:
			self._serial = serial.Serial()
			self._serial.port = port
			self._serial.baudrate = 115200
			self._serial.setDTR(False)
			self._serial.open()

		self._mode = ProtoModes.OFF

		# Send end of packet marker to flush its buffer
		self._serial.write(b'\x00')

		# Set to off
		self.start_mode(ProtoModes.OFF)

	def close(self):
		# Make sure it is off before actually closing
		self.finish()
		self._serial.close()

	def start_mode(self, mode):
		self._write_packet(RequestId.MODE, 'B', mode.value)

		devId, = self._read_struct(ReplyId.DEVICE_ID, 'H')
		if mode != ProtoModes.OFF and devId != 0xAA1:
			self._mode = ProtoModes.OFF
			raise Exception("Unsupported device ID: %06X" % devId)

		self._mode = mode

	def start_read(self):
		devId = self.start_mode(ProtoModes.READ)

	def read(self, offset, length):
		if self._mode is not ProtoModes.READ:
			raise Exception("Not in read mode")

		words = array.array('H')
		while length > 0:
			iterlen = min(length, Programmer.MAX_WORDS)
			self._write_packet(RequestId.READ, 'HB', offset, iterlen)

			iterdata = self._read_expected(ReplyId.READ)

			if len(iterdata) != iterlen * 2:
				raise Exception("Expected to read %d bytes, got %d" % (iterlen * 2, len(iterdata)))

			words.frombytes(iterdata)
			offset += iterlen
			length -= iterlen

		return words

	def start_write(self):
		devId = self.start_mode(ProtoModes.WRITE)

	def write(self, offset, words):
		if self._mode != ProtoModes.WRITE:
			raise Exception("Not in write mode")

		if len(words) % 4 != 0:
			raise Exception("Words must be programmed in chunks of 4")

		bytes = words.tobytes()
		while len(bytes) > 0:
			iterlen = min(len(bytes), Programmer.MAX_WORDS * 2)
			iterdata = bytes[0:iterlen]
			self._write_packet(RequestId.WRITE, 'H', offset, payload=iterdata)
			self._read_struct(ReplyId.OK)

			bytes = bytes[iterlen:]

	def erase(self):
		self.start_mode(ProtoModes.ERASE)
		self._write_packet(RequestId.ERASE)
		self._read_struct(ReplyId.OK)

	def finish(self):
		if self._mode != ProtoModes.OFF:
			self.start_mode(ProtoModes.OFF)

	def _write_packet(self, packet_type, packet_format='', *packet_data, payload=b''):
		packet = struct.pack('<B' + packet_format, packet_type.value, *packet_data) + payload
		print('> %s' % packet.hex())
		packet = cobs.encode(packet) + b'\x00'
		self._serial.write(packet)

	def _read_packet(self, timeout=0.5):
		self._serial.timeout = timeout
		reply = self._serial.read_until(b'\x00')
		if len(reply) == 0:
			raise Exception('No packet received')

		if reply[-1] != 0:
			raise Exception('Unterminated packet')

		reply = cobs.decode(reply[0:-1])
		print('< %s' % reply.hex())
		return reply

	def _read_expected(self, expectedType):
		reply = self._read_packet()
		if len(reply) == 0:
			raise Exception('Received empty packet')

		if reply[0] != expectedType.value:
			raise Exception('Expected %s, got packet %d' % (expectedType, reply[0]))

		return reply[1:]

	def _read_struct(self, expectedType, expectedFormat=''):
		reply = self._read_expected(expectedType)

		try:
			unpacked = struct.unpack('<' + expectedFormat, reply)
		except:
			raise Exception('Received packet does not match expected format')

		return unpacked

	def __enter__(self):
		return self

	def __exit__(self, exc_type, exc_val, exc_tb):
		self.close()
