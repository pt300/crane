#include <Arduino.h>
#include "PIDF.h"

PIDF::PIDF(float Kp, float Ki, float Kd, float saturation, Filter& filt, float frequency) : Kp(Kp), Ki(Ki), Kd(Kd), saturation(saturation), frequency(frequency), filt(filt) {
  //filt = f(Cf, frequency);
  reset();
}

void PIDF::reset() {
  //reset integral
  xd = 0;
  xdf = 0;
  integral = 0;
  filt.frequency = frequency;
  filt.reset();
}

float PIDF::process(float sample) {
  integral += sample / frequency;
  integral = constrain(integral, -saturation, saturation); //anti windup ?
  
  float sample_filt = filt.process(sample) * frequency;
  float out = sample * Kp + integral * Ki + (xdf - sample_filt) * Kd;

  xd = sample;
  xdf = sample_filt;
  
  return out;
}
