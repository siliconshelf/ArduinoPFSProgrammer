
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
#define SMPS_PROG_VOLTS SMPS_ADC_VAL(6)
#define SMPS_ERASE_VOLTS SMPS_ADC_VAL(6.5)

#define BASEDELAY 1

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

  Serial.print(F("Target: "));
  Serial.println(volts, HEX);
  
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

void padauk_begin(uint8_t volts) {
  smpsOn(volts);
  delayMicroseconds(100);

  digitalWrite(VDD33_EN, LOW);
  delayMicroseconds(500);
}

void padauk_finish() {
  digitalWrite(VDD33_EN, HIGH);
  digitalWrite(VDDSMPS_EN, LOW);
  smpsOff();
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

void setup() {
  uint16_t ack = 0;
  
  // put your setup code here, to run once:
  pinMode(11, OUTPUT);

  pinMode(VDD33_EN, OUTPUT);
  digitalWrite(VDD33_EN, HIGH);
  
  pinMode(SMPS_PIN, OUTPUT);
  pinMode(VDDSMPS_EN, OUTPUT);
  pinMode(PROG_DATA, OUTPUT);
  pinMode(PROG_CLOCK, OUTPUT);
  
  cli();
  smpsInit();
  sei();

  Serial.begin(115200);

#if 0
  // ERASE TEST
  padauk_begin(SMPS_PROG_VOLTS);
  ack = padauk_command(3);
  Serial.println(ack, HEX);
  
  padauk_begin(SMPS_ERASE_VOLTS);
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

  delay(100);
#endif

  // WRITE TEST
#if 0
  padauk_begin(SMPS_PROG_VOLTS);
  ack = padauk_command(7);
  //smpsOn(SMPS_83V);

  pinMode(VDD33_EN, INPUT);
  pinMode(VDDSMPS_EN, OUTPUT);
  digitalWrite(VDDSMPS_EN, LOW);
  
  Serial.println(ack, HEX);
  for (uint16_t addr = 0; addr < 2048; addr += 4) {
    uint16_t words[4] = {0x0000 + addr, addr + 1, addr + 2, addr + 3};
    padauk_flash_write(addr, words);
  }
  
  pinMode(VDDSMPS_EN, INPUT);
  padauk_finish();

  delay(100);
#endif


  // READ TEST
  padauk_begin(SMPS_PROG_VOLTS);
  ack = padauk_command(6);

#define SERIALLOG

#ifdef SERIALLOG
  Serial.println(ack, HEX);
#endif

#if 1
  for (uint16_t addr = 0; addr < 2048; addr++) {
    uint16_t data = padauk_flash_read(addr);
    if (data < 0x1000) {
      Serial.print('0');
      if (data < 0x100) {
        Serial.print('0');
        if (data < 0x10) {
          Serial.print('0');
        }
      }
    }
    Serial.print(data, HEX);
    if ((addr & 15) == 15) {
      Serial.println();
    } else {
      Serial.print(' ');
    }
  }
#endif
  padauk_finish();
}

void loop() {
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
