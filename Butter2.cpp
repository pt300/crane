#include <Arduino.h>
#include "Butter2.h"

Butter2::Butter2(float frequency, float cutoff) : Filter(frequency, cutoff) {
  reset();  
}

void Butter2::reset() {
    xd1 = 0;
    xd2 = 0;
    yd1 = 0;
    yd2 = 0;
    
    float A = 1 / tan(cutoff * PI / frequency);
    float DENOM = A*A + A * 2 / sqrt(2) + 1;

    cx02 = 1 / DENOM;
    cx1 = 2 / DENOM;
    cy1 = - (2.f-2*A*A) / DENOM;
    cy2 = (A*A - A * 2 / sqrt(2) + 1) / DENOM;
}

float Butter2::process(float sample) {
  float out;

  out = cx02 * (sample + xd2) + cx1 * xd1 + cy1 * yd1 + cy2 * yd2;
  
  xd2 = xd1;
  xd1 = sample;
  yd2 = yd1;
  yd1 = out;

  return out;
}
