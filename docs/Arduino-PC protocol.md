
Arduino-PC protocol
===================

This document explains the communication protocol employed by the PC to talk to the firmware running on the Arduino.

Hardware layer
--------------

The communication is done on standard serial ([UART](https://en.wikipedia.org/wiki/Universal_asynchronous_receiver-transmitter)), using a 115200 baud rate with one stop bit and no parity.

The PC client should ensure that the DTR line is not asserted when opening the connection to the Arduino, since this asynchronous line is connected to the reset pin of the Atmega, causing the chip to reset and communication to fail.

Framing layer
-------------

On top of UART, all the data is framed into packets using the standard framing protocol known as Consistent Overhead Byte Stuffing, or [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing). COBS removes all null/zero bytes (`0x00`) from the data packets, and replaces them with non-null bytes.

This enables us to use null bytes as special markers for packet boundaries.

The procedure to send a data packet is:

 - Run the data packet through the encode function of COBS, which will return a stuffed packet with one extra byte at the head and no null bytes.
 - Send the microcontroller the stuffed packet.
 - Send the microcontroller a null byte to mark the end of packet.

For receiving a packet, the algorithm should be executed as follows:
 - Read until a null byte is found.
 - Remove trailing null byte from buffer.
 - Run the stuffed through the decode function of COBS, which will return the original unstuffed packet that may contain null bytes.

The microcontroller will ignore any empty packets. The PC however should handle this as an error, since the microcontroller is never supposed to generate such packets.

It is recommended to send one null byte at the start of the programming sequence, to ensure the read buffer in the microcontroller is empty when the next packet is sent. If the buffer was already empty, no error condition will arise given this would be decoded as an empty packet and thus ignored.

Packet format
-------------

Each packet is formed by a prefix byte indicating the type, followed by the payload, if any. All the multi-byte values (such as words) are stored in little endian format (LSB first).

### Requests (from PC to Arduino)

#### `REQUEST_MODE` (id: `00`)

This packet is employed to change the programmer mode between the several supported modes.

```
+-00-+-01-+
| 00 | mm |
+----+----+
```

##### Parameters:

 - `mm`: mode ID.
   The supported mode IDs are:
	 - `00`: off mode. The microcontroller is off, with no power being delivered to its VCC pin.
	 - `01`: read mode. The programmer is ready to relay read commands.
	 - `02`: write mode. The programmer is ready to relay write commands.
	 - `03`: erase mode. The programmer is ready to proceed with erasing the chip.

##### Replies:

- `REPLY_OK`: the mode is off and the microcontroller has been shutdown.
- `REPLY_DEVICE_ID`: the mode is not off, a microcontroller has been detected and the requested mode has been entered successfully.
- `REPLY_NO_CHIP`: the mode is not off, and the microcontroller was not detected. The programmer is now in off mode.
- `REPLY_INVALID_REQUESTED_MODE`: the mode does not exist. The programmer is now in off mode.

#### `REQUEST_READ` (id: `0x01`)

This packet is employed to read up to 64 words of data from the microcontroller's non-volatile flash memory. The programmer must have been previously put in read mode.

```
+-00-+-01-+-02-+-03-+
| 00 | aa | aa | ll |
+----+----+----+----+
```

##### Parameters:

 - `aaaa`: start word (offset).
 - `ll`: length in words to read, up to 64.

##### Replies:

 - `REPLY_READ`: the requested memory chunk.
 - `REPLY_INVALID_CURRENT_MODE`: the current mode is not read.
 - `REPLY_CHUNK_TOO_LARGE`: the length parameter exceeded 64 words.

#### `REQUEST_WRITE` (id: `0x02`)

This packet writes up to 64 bytes on the microcontroller's non-volatile flash memory. The programmer must have been previously put in write mode.

```
+-00-+-01-+-02-+-03-+-04-+-05-+-06-+-07-+-08-+-..
| 00 | aa | aa | w1 | w1 | w2 | w2 | w3 | w3 | ..
+----+----+----+----+----+----+----+----+----+-..
```

##### Parameters:

 - `aaaa`: start word (offset).
 - `wnwn`: data to write, up to 64 words.
 
##### Replies:

 - `REPLY_OK`: the data has been successfully relayed to the microcontroller. However, due to the impossibility to read data while the target microcontroller is in write mode, the programmer is unable to verify the written data and thus is __no guarantee that the data has been written correctly__. It is recommended to verify the data by reading it back after all the data chunks have been written.
 - `REPLY_INVALID_CURRENT_MODE`: the current mode is not write.

#### `REQUEST_ERASE` (id: `0x03`)

This packet continues with the erase sequence previously started by setting the erase mode, resetting all the non-volatile flash memory back to its factory status.

```
+-00-+
| 03 |
+----+
```

##### Parameters:

None.

##### Replies:

 - `REPLY_OK`: the sequence has finished successfully.
 - `REPLY_INVALID_CURRENT_MODE`: the current mode is not write.

### Replies (from Arduino to PC)

#### `REPLY_OK` (id: `0x00`)

This reply indicates that the requested command has been successfully executed.

```
+-00-+
| 00 |
+----+
```

##### Data:

None

#### `REPLY_DEVICE_ID` (id: `0x01`)

This reply contains the detected device ID.

```
+-00-+-01-+-02-+
| 01 | ii | ii |
+----+----+----+
```

##### Data:

 - `iiii`: the 12-bit device ID, returned by the target microcontroller after entering any mode.

#### `REPLY_READ` (id: `0x02`)

This reply contains the read data from the non-volatile flash memory.

```
+-00-+-01-+-02-+-03-+-04-+-05-+-06-+-..
| 02 | w1 | w1 | w2 | w2 | w3 | w3 | ..
+----+----+----+----+----+----+----+-..
```

##### Data:

 - `wnwn`: read data, up to 64 words.

#### `REPLY_UNKNOWN_REQUEST` (id: `0x80`)

This is an error which means that an unknown request has been issued to the Arduino.

```
+-00-+
| 80 |
+----+
```

##### Data:

None

#### `REPLY_INVALID_REQUEST_LENGTH` (id: `0x81`)

This is an error which means that a request with a supported type id has been issued, but that the overall packet size of it does not match that of a valid request.

```
+-00-+
| 81 |
+----+
```

##### Data:

None

#### `REPLY_INVALID_REQUESTED_MODE` (id: `0x82`)

The PC has requested the Arduino to enter an unsupported mode, and has changed to off mode instead.

```
+-00-+
| 82 |
+----+
```

##### Data:

None

#### `REPLY_INVALID_CURRENT_MODE` (id: `0x83`)

The PC has issued a fully valid request to the Arduino, but the Arduino is unable to execute it in its current mode.

```
+-00-+
| 83 |
+----+
```

##### Data:

None

#### `REPLY_CHUNK_TOO_LARGE` (id: `0x84`)

The PC has requested a read whose size exceeds the supported maximum.

```
+-00-+
| 84 |
+----+
```

##### Data:

None

#### `REPLY_NO_CHIP` (id: `0x85`)

This is an error signaling that no chip has been detected, and that the programmer has switched to off mode.

```
+-00-+
| 85 |
+----+
```

##### Data:

None
