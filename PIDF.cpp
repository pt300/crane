#include <Arduino.h>
#include "PIDF.h"

PIDF::PIDF(float Kp, float Ki, float Kd, float Kb, float saturation, Filter& filt, float frequency) : Kp(Kp), Ki(Ki), Kd(Kd), Kb(Kb), saturation(saturation), frequency(frequency), filt(filt) {
  //filt = f(Cf, frequency);
  reset();
}

void PIDF::reset() {
  //reset integral
  yd = 0;
  xd = 0;
  xdf = 0;
  integral = 0;
  filt.frequency = frequency;
  filt.reset();
}

float PIDF::process(float sample) {
  float constrained = constrain(yd, -saturation, saturation);
  float backcalc = constrained - yd;
  //backcalc * Kb +
  integral += (Kb * backcalc + Ki * sample) / frequency;
  //integral = constrain(integral, -saturation, saturation);

  float sample_filt = filt.process(sample) * frequency;
  float derivative = (sample_filt - xdf) * Kd;
  
  float out = sample * Kp + integral + derivative;

  xd = sample;
  xdf = sample_filt;

  yd = out;
  return out;
}
