#include "PENDULUM.h"
#include <SPI.h>

#include "debug.h"

PENDULUM::PENDULUM() : ss(53) { }

void PENDULUM::begin() {
  pinMode(ss, OUTPUT);
  digitalWrite(ss, HIGH);
  SPI.begin();
}

void PENDULUM::zero() {
  /*
   * Loop that polls position of the poendulum
   * and makes sure the same position stays for a while
   */
  DMSGLN("[PENDULUM] Waiting for pendulum to settle");
  int hits = 0;
  int16_t last = 0;
  while(hits != 5) {
    int16_t now = read_raw();
    if(now == last) {
      hits++;
    }
    else {
      hits = 0;
    }

    last = now;
    delay(50);
  }
  DMSGLN("[PENDULUM] Pendulum settled, zeroing");
  
  transaction(commands::ZERO);
  delay(200); //startup time
}

float PENDULUM::read() {
  float out = read_raw();
  out *= PI;
  out /= 0x1FFF;

  return out;
}

int16_t PENDULUM::read_raw() {
  int16_t value = transaction(commands::READ);
  value &= 0x3FFF; //discard parity bits

  if(value & 0x2000) { //sign extension to 16bit
    value |= 0xC000;
  }
  else {
    value &= 0x3FFF;
  }

  return value;
}

int16_t PENDULUM::transaction(enum commands command) {
  char buf[2] = {0x00, command};
  
  DMSGLN("[PENDULUM] Sending a command: " + command);
  
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  
  digitalWrite(ss, LOW);
  delayMicroseconds(3);
  SPI.transfer(&buf[0], 1 , SPI_CONTINUE);
  delayMicroseconds(3); //required delay, does not work properly without it
  SPI.transfer(&buf[1], 1, SPI_LAST);
  SPI.endTransaction();
  delayMicroseconds(3);
  digitalWrite(ss, HIGH);
  
  DMSGLN("[PENDULUM] Transfer finished");
  int16_t out = buf[1];
  out |= buf[0] << 8;

  return out;
}
