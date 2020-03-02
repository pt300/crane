#include <arduino.h>
#include "MOTOR.h"

#define DEBUG
#include "debug.h"

MOTOR::MOTOR() {}

void MOTOR::begin() {
  pmc_set_writeprotect(false);

  pmc_enable_periph_clk(ID_TC0);

  /* 
   * TIOA0 to PB25 (Pin 2), PERIPH_B
   */
  PIO_Configure(PIOB, PIO_PERIPH_B, PIO_PB25B_TIOA0, PIO_DEFAULT);

/* 
 * MCK / 2 as clock source
 * Up count with reset on RC compare
 * Waveform mode
 * Clear on RA compare
 * Set on RC compare
 */
  TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET | TC_CMR_TCCLKS_TIMER_CLOCK1);

  TC_SetRA(TC0, 0, 1400);
  TC_SetRC(TC0, 0, 2800);

  TC_Start(TC0, 0);

  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  DMSGLN("[MOTOR] begin()");
}

void MOTOR::write_duty(int duty) {
  duty = constrain(duty, 1, 2800); //flip
  
  DMSGLN("[MOTOR] Setting RA: " + duty);
  TC_SetRA(TC0, 0, duty);
}

void MOTOR::write_voltage(float voltage) {
  int out = (voltage + 1) * 1400;
  
  Serial.print("[MOTOR] Requested voltage: " + voltage);
  write_duty(out);
}
