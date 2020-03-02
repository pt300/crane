#include <arduino.h>
#include "CART.h"

#define DEBUG
#include "debug.h"

CART::CART(MOTOR& motor) : motor(motor), left_side(0), right_side(1) { }

void CART::begin() {
  pmc_set_writeprotect(false);

  pmc_enable_periph_clk(ID_TC6);
  pmc_enable_periph_clk(ID_TC7);

  /* 
   * 36.6.14.1
   * The quadrature decoder (QDEC) is driven by TIOA0, TIOB0, TIOB1 input pins and 
   * drives the timer/counter ofchannel 0 and 1.
   * 
   * Connect TIOA6 and TIOB6 of TC2. Using CART here. TIOB7 is index signal, unused here.
   * 
   * PIO_PULLUP ??
   */
  PIO_Configure(PIOB, PIO_PERIPH_B, PIO_PC25B_TIOA6 | PIO_PC26B_TIOB6, PIO_DEFAULT);

/* 
 * 36.6.14.1 Description
 * Field TCCLKS of TC_CMRx must be configured to select XC0 input (i.e., 0x101).
 * Field TC0XC0S has no effect assoon as the QDEC is enabled.
 * 36.6.14.4 Position and Rotation Measurement
 * Channel 0 and 1 must be configured in Capture mode (TC_CMR0.WAVE = 0).
 * ‘Rising edge’ must be selected asthe External Trigger Edge (TC_CMR.ETRGEDG = 0x01) and
 * ‘TIOA’ must be selected as the External Trigger(TC_CMR.ABETRG = 0x1).
 */
  TC_Configure(TC2, 0, TC_CMR_ETRGEDG_RISING | TC_CMR_ABETRG | TC_CMR_TCCLKS_XC0);
  TC_Configure(TC2, 1, TC_CMR_ETRGEDG_RISING | TC_CMR_ABETRG | TC_CMR_TCCLKS_XC0);

/*
 * 36.6.14.4 Position and Rotation Measurement
 * When the POSEN bit is set in the TC_BMR, the motor axis position is processed on channel 0
 * (by means of the PHA, PHB edge detections) and the number of motor revolutions are recorded
 * on channel 1 if the IDX signal isprovided on the TIOB1 input. The position measurement can
 * be read in the TC_CV0 register and the rotationmeasurement can be read in the TC_CV1 register.
 * 
 * 36.7.14 TC Block Mode Register
 * QDEN: Enables the QDEC (filter, edge detection and quadrature decoding)
 * POSEN: Enables the position measure on channel 0 and 1
 * QDTRAN: Quadrature decoding logic is inactive (direction change inactive) but input filtering and edge detection are performed
 * EDGPHA: Edges are detected on both PHA and PHB
 */
  TC2->TC_BMR = TC_BMR_EDGPHA | /*TC_BMR_QDTRANS | */TC_BMR_POSEN | TC_BMR_QDEN;

  TC_Start(TC2, 0);
  TC_Start(TC2, 1);
  
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  DMSGLN("[CART] begin()");
}

int32_t CART::read_raw() {
  return TC_ReadCV(TC2, 0);
}

float CART::read() {
  float out = read_raw() - left_side;
  out /= right_side - left_side;
  out = (out * 2) - 1;

  return out;
}

void CART::calibrate(float slow, float fast) {
  DMSGLN("[CART] Looking for left edge");
  
  reach(slow, fast, LEFT);
  left_side = read_raw();
  DMSGLN("[CART] Left edge at: " + left_side);
  
  delay(500);
  DMSGLN("[CART] Looking for right edge");
  reach(slow, fast, RIGHT);
  right_side = read_raw();
  DMSGLN("[CART] Right edge at: " + right_side);
  
  DMSGLN("[CART] Moving off the switch");
  motor.write_voltage(-slow);
  while(digitalRead(7) == HIGH);
  delay(100);
  motor.write_voltage(0);
  
}

void CART::move_to(float position, float P) {
  position = constrain(position, -1, 1);
  P = -1 * abs(P); //make sure we will go the right way
  float current = read();
  
  while(abs(position - current) >= 0.001 &&
        digitalRead(6) == LOW &&
        digitalRead(7) == LOW) {
    current = read();
    motor.write_voltage(current * P);
  }
  motor.write_voltage(0);
}

/*
 * Reach a switch on a given side. Tries to do precise
 * slow positioning after reaching the switch.
 */
void CART::reach(float slow, float fast, enum CART::side side) {
  int switch_pin;
  
  switch(side) {
    case LEFT:
      switch_pin = 6;
      slow *= -1;
      fast *= -1;
      break;
    case RIGHT:
      switch_pin = 7;
      break;
    default:
    return;
  }

  DMSGLN("[CART] Switch pin: " + switch_pin);
  
  DMSGLN("[CART] Going towards switch");
  //reach the switch first
  motor.write_voltage(fast);
  int kill = digitalRead(switch_pin);
  while(kill == LOW) {
    kill = digitalRead(switch_pin);
  }
  motor.write_voltage(0);
  DMSGLN("[CART] Switch reached");
  
  delay(500);
  DMSGLN("[CART] Going slightly away from switch");
  motor.write_voltage(-1 * slow);
  while(digitalRead(switch_pin) == HIGH);
  delay(100);
  motor.write_voltage(0);
  DMSGLN("[CART] Moved away from switch");
  
  delay(500);
  DMSGLN("[CART] Going back slowly towards switch");
  motor.write_voltage(slow);
  while(digitalRead(switch_pin) == LOW);
  motor.write_voltage(0);
  DMSGLN("[CART] Switch reached");

}
