
#include <PacketSerial.h>

// SMPS clock output pin
#define SMPS_PORT PORTD
#define SMPS_BIT 3

// SMPS feedback ADC
#define SMPS_ADC 0

// Current target voltage
uint8_t smpsTarget = 0;

// Set when the SMPS has a stable voltage
volatile boolean smpsReachedLower;
volatile boolean smpsReachedHigher;

// 470ohm and 47ohm are the voltage divisors
// 1.1 being the internal 1.1V bandgap reference
#define SMPS_DIV_R1 680.0
#define SMPS_DIV_R2 100.0
#define SMPS_ADC_VAL(v) ((uint8_t) ((255 * v * SMPS_DIV_R2) / (1.1 * (SMPS_DIV_R2 + SMPS_DIV_R1))))

#define BASEDELAY 3

void smpsInit() {
  // - Set channel to ADC0 (MUX[3:0] = 0000)
  // - Enable 1.1V input reference (REFS[1:0] = 11)
  // - Let left adjust (ADLAR = 1)
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(ADLAR) | (SMPS_ADC << MUX0);

  // - Enable free running mode (ADTS = 0)
  ADCSRB = 0;

  smpsOff();
}

void smpsOn(uint16_t volts) {
  smpsTarget = volts;

  cli();
  smpsReachedHigher = false;
  smpsReachedLower = false;

  // - Enable ADC (ADEN = 1)
  // - Start conversion (ADSC = 1)
  // - Auto trigger enable (ADATE = 1)
  // - Enable interrupts (ADIE = 1)
  // - Set prescaler to 16 for 1MHz ADC clock (ADPS[2:0] = 100)
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2);
  sei();

  while (smpsReachedLower == false || smpsReachedHigher == false);
}

void smpsOff() {
  // - Disable ADC (ADEN = 0)
  // - Disable autotrigger (ADATE = 0)
  // - Disable interrupts (ADIE = 0)
  ADCSRA = 0x00;
  SMPS_PORT &= ~_BV(SMPS_BIT);
}

#define SMPS_PIN 3
#define VDD33_EN 4
#define VDDSMPS_EN 2
#define PROG_DATA 5
#define PROG_CLOCK 6
#define PROG_VOLTS SMPS_ADC_VAL(6.5)
#define ERASE_VOLTS SMPS_ADC_VAL(8)

void padauk_init() {
  pinMode(VDD33_EN, OUTPUT);
  digitalWrite(VDD33_EN, HIGH);
  
  pinMode(SMPS_PIN, OUTPUT);
  pinMode(VDDSMPS_EN, OUTPUT);
  pinMode(PROG_DATA, OUTPUT);
  pinMode(PROG_CLOCK, OUTPUT);

  smpsInit();
}

void padauk_spi_clock() {
  digitalWrite(PROG_CLOCK, HIGH);
  delayMicroseconds(BASEDELAY);
  digitalWrite(PROG_CLOCK, LOW);
  delayMicroseconds(BASEDELAY);
}

void padauk_spi_input() {
  digitalWrite(PROG_DATA, LOW);
  pinMode(PROG_DATA, INPUT);
  delayMicroseconds(BASEDELAY);
}

void padauk_spi_output() {
  digitalWrite(PROG_DATA, LOW);
  pinMode(PROG_DATA, OUTPUT);
  delayMicroseconds(BASEDELAY);
}

void padauk_spi_write_bit(int bit) {
  digitalWrite(PROG_DATA, bit ? HIGH : LOW);
  delayMicroseconds(BASEDELAY);
  padauk_spi_clock();
}

void padauk_spi_write(uint16_t data, uint8_t bits) {
  do {
    bits--;
    padauk_spi_write_bit(data & (1 << bits));
  } while (bits > 0);
}

boolean padauk_spi_read_bit() {
  digitalWrite(PROG_CLOCK, HIGH);
  delayMicroseconds(BASEDELAY);
  boolean val = digitalRead(PROG_DATA);
  digitalWrite(PROG_CLOCK, LOW);
  delayMicroseconds(BASEDELAY);
  return val;
}

uint16_t padauk_spi_read(uint8_t bits) {
  uint16_t data = 0;
  while (bits > 0) {
    data = data << 1 | padauk_spi_read_bit();
    bits--;
  }
  return data;
}

uint16_t padauk_command(uint8_t cmd) {
  padauk_spi_write( 0xA5A, 12);
  padauk_spi_write(0x5A5A, 16);
  padauk_spi_write(cmd, 4);

  padauk_spi_input();

  padauk_spi_clock();
  padauk_spi_clock();
  padauk_spi_clock();
  padauk_spi_clock();

  uint16_t ack = padauk_spi_read(12);

  padauk_spi_clock();
  padauk_spi_output();
  return ack;
}

uint16_t padauk_start(uint8_t cmd) {
  smpsOn(PROG_VOLTS);
  delayMicroseconds(100);

  digitalWrite(VDD33_EN, LOW);
  delayMicroseconds(500);

  return padauk_command(cmd);
}

void padauk_finish() {
  digitalWrite(VDD33_EN, HIGH);
  digitalWrite(VDDSMPS_EN, LOW);
  smpsOff();
  delay(100);
}

uint16_t padauk_flash_read(uint16_t addr) {
  padauk_spi_write(addr, 13);
  padauk_spi_input();
  uint16_t data = padauk_spi_read(14);
  padauk_spi_output();
  padauk_spi_clock();
  return data;
}

void padauk_flash_write(uint16_t addr, const uint16_t * data) {
  padauk_spi_write(data[0], 14);
  padauk_spi_write(data[1], 14);
  padauk_spi_write(data[2], 14);
  padauk_spi_write(data[3], 14);
  padauk_spi_write(addr, 13);
  padauk_spi_write(0, 9);
}

void padauk_erase() {
  smpsOn(ERASE_VOLTS);
  delayMicroseconds(10000);
  digitalWrite(PROG_CLOCK, HIGH);
  delayMicroseconds(5000);
  digitalWrite(PROG_CLOCK, LOW);
  digitalWrite(PROG_CLOCK, HIGH);
  digitalWrite(PROG_CLOCK, LOW);
  digitalWrite(PROG_CLOCK, HIGH);
  delayMicroseconds(5000);
  digitalWrite(PROG_CLOCK, LOW);
  digitalWrite(PROG_CLOCK, HIGH);
  digitalWrite(PROG_CLOCK, LOW);
  delayMicroseconds(150);
  padauk_finish();
}

enum {
  REQUEST_MODE = 0,
  REQUEST_READ,
  REQUEST_WRITE,
  REQUEST_ERASE,
};

enum {
  MODE_OFF = 0,
  MODE_READ,
  MODE_WRITE,
  MODE_ERASE,
};

struct request_mode {
  uint8_t mode;
};

struct request_read {
  uint16_t address;
  uint8_t len;
};

struct request_write {
  uint16_t address;
  uint16_t data[0];
};

struct request {
  uint8_t type;
  union {
    struct request_mode mode;
    struct request_read read;
    struct request_write write;
  };
};

enum {
  REPLY_OK = 0,
  REPLY_DEVICE_ID,
  REPLY_READ,
  REPLY_UNKNOWN_REQUEST = 0x80,
  REPLY_NO_CHIP,
  REPLY_INVALID_REQUEST_LENGTH,
  REPLY_INVALID_REQUESTED_MODE,
  REPLY_INVALID_CURRENT_MODE,
  REPLY_CHUNK_TOO_LARGE,
};

struct reply_device_id {
  uint16_t device_id;
};

struct reply_read {
  uint16_t data[64];
};

struct reply {
  uint8_t type;
  union {
    struct reply_device_id device_id;
    struct reply_read read;
  };
};

PacketSerial packetSerial;
uint8_t currentMode = MODE_OFF;
bool setupWrite;

void onPacketReceived(const uint8_t * packet, size_t len) {
  if (len == 0) {
    return;
  }

  const struct request * request = (const struct request *) packet;
  struct reply reply;

  switch (request->type) {
    case REQUEST_MODE:
      if (len != 1 + sizeof(struct request_mode)) {
        reply.type = REPLY_INVALID_REQUEST_LENGTH;
        packetSerial.send((uint8_t *) &reply, 1);
        return;
      }
  
      if (currentMode != MODE_OFF) {
        padauk_finish();
        currentMode = MODE_OFF;
      }
  
      reply.device_id.device_id = 0x000;
      switch (request->mode.mode) {
        case MODE_OFF:
          reply.type = REPLY_OK;
          packetSerial.send((uint8_t *) &reply, 1);
          return;
  
        case MODE_READ:
          reply.device_id.device_id = padauk_start(0x06);
          break;
  
        case MODE_WRITE:
          reply.device_id.device_id = padauk_start(0x07);
          setupWrite = false;
          break;
  
        case MODE_ERASE:
          reply.device_id.device_id = padauk_start(0x03);
          break;
  
        default:
          reply.type = REPLY_INVALID_REQUESTED_MODE;
          packetSerial.send((uint8_t *) &reply, 1);
          return;
      }
  
      if (reply.device_id.device_id == 0x000) {
        padauk_finish();
        reply.type = REPLY_NO_CHIP;
        packetSerial.send((uint8_t *) &reply, 1);
      }
  
      currentMode = request->mode.mode;
      reply.type = REPLY_DEVICE_ID;
      packetSerial.send((uint8_t *) &reply, 1 + sizeof(reply_device_id));
      return;

    case REQUEST_READ:
      if (len != 1 + sizeof(struct request_read)) {
        reply.type = REPLY_INVALID_REQUEST_LENGTH;
        packetSerial.send((uint8_t *) &reply, 1);
        return;
      }

      if (currentMode != MODE_READ) {
        reply.type = REPLY_INVALID_CURRENT_MODE;
        packetSerial.send((uint8_t *) &reply, 1);
        return;
      }

      if (request->read.len > sizeof(reply.read.data) / sizeof(uint16_t)) {
        reply.type = REPLY_CHUNK_TOO_LARGE;
        packetSerial.send((uint8_t *) &reply, 1);
        return;
      }

      reply.type = REPLY_READ;
      for (uint8_t i = 0; i < request->read.len; i++) {
        reply.read.data[i] = padauk_flash_read(request->read.address + i);
      }
      packetSerial.send((uint8_t *) &reply, 1 + request->read.len * sizeof(uint16_t));
      return;

    case REQUEST_WRITE: {
      // Subtract request id and offset
      int16_t payloadLen = len - sizeof(uint8_t) - sizeof(uint16_t);
      if (payloadLen < 0 || (payloadLen % sizeof(uint16_t)) != 0) {
        reply.type = REPLY_INVALID_REQUEST_LENGTH;
        packetSerial.send((uint8_t *) &reply, 1);
        return;
      }

      if (!setupWrite) {
        digitalWrite(VDD33_EN, HIGH);
        digitalWrite(VDDSMPS_EN, HIGH);
        delay(100);
        setupWrite = true;
      }

      uint8_t words = payloadLen / sizeof(uint16_t);
      uint8_t pos = 0;
      while (words >= 4) {
        padauk_flash_write(request->write.address + pos, request->write.data + pos);
        pos += 4;
        words -= 4;
      }

      if (words != 0) {
        uint16_t tmp[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
        memcpy(tmp, request->write.data + pos, words * sizeof(uint16_t));
        padauk_flash_write(request->write.address + pos, tmp);
      }

      reply.type = REPLY_OK;
      packetSerial.send((uint8_t *) &reply, 1);
      return;
    }

    case REQUEST_ERASE:
      if (len != 1) {
        reply.type = REPLY_INVALID_REQUEST_LENGTH;
        packetSerial.send((uint8_t *) &reply, 1);
        return;
      }

      if (currentMode != MODE_ERASE) {
        reply.type = REPLY_INVALID_CURRENT_MODE;
        packetSerial.send((uint8_t *) &reply, 1);
        return;
      }

      padauk_erase();
      currentMode = MODE_OFF;

      reply.type = REPLY_OK;
      packetSerial.send((uint8_t *) &reply, 1);
      return;
  }

  reply.type = REPLY_UNKNOWN_REQUEST;
  packetSerial.send((uint8_t *) &reply, 1);
}

void setup() {
  padauk_init();
  packetSerial.begin(115200);
  packetSerial.setPacketHandler(&onPacketReceived);
}

void loop() {
  packetSerial.update();
  if (packetSerial.overflow()) {
    uint8_t reply = REPLY_INVALID_REQUEST_LENGTH;
    packetSerial.send(&reply, 1);
  }
}

ISR(ADC_vect) {
  if (SMPS_PORT & _BV(SMPS_BIT)) {
    SMPS_PORT &= ~_BV(SMPS_BIT);
  } else {
    if (ADCH >= smpsTarget) {
      smpsReachedHigher = true;
    } else {
      smpsReachedLower = true;
      SMPS_PORT |= _BV(SMPS_BIT);
    }
  }
}
