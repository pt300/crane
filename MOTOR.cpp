#include <arduino.h>
#include "MOTOR.h"

#include "debug.h"

#define TOP  2800 //8600
#define MID (TOP/2)

MOTOR::MOTOR(float voltage) : voltage_max(voltage), voltage_limit(voltage) {}

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

//  TC_SetRA(TC0, 0, 1400);
//  TC_SetRC(TC0, 0, 2800);

  TC_SetRA(TC0, 0, MID);
  TC_SetRC(TC0, 0, TOP);

  TC_Start(TC0, 0);

  pinMode(3, OUTPUT);
  digitalWrite(3, LOW); //keep disabled

  DMSGLN("[MOTOR] begin()");
}

void MOTOR::write_duty(int duty) {
  duty = constrain(duty, 1, TOP); //flip
  
  DMSGLN("[MOTOR] Setting RA: " + duty);
  TC_SetRA(TC0, 0, duty);
}

void MOTOR::write_voltage(float voltage) {
  voltage = constrain(voltage, -voltage_limit, voltage_limit);
  int out = (voltage / voltage_max + 1) * MID;
  
  DMSGLN("[MOTOR] Requested voltage: " + voltage);
  write_duty(out);
  
  digitalWrite(3, voltage != 0); //turns off driver completely for 0 output, prevents annoying noise
}

void MOTOR::enable(bool e) {
  write_voltage(0);
  digitalWrite(3, e);
}
/*
void MOTOR::write_voltage_unscaled(float voltage) {
  int out = (voltage + 1) * 1400;
  
  DMSGLN("[MOTOR] Requested voltage: " + voltage);
  write_duty(out);
}*/
