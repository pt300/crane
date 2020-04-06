#include <Arduino.h>
#include "Butter1.h"

Butter1::Butter1(float frequency, float cutoff) : Filter(frequency, cutoff) {
  reset();  
}

void Butter1::reset() {
    xd1 = 0;
    yd1 = 0;

    float A = 1 / tan(cutoff * pi / frequency);
    cx = 1 / (A + 1);
    cy = (A - 1) / (A + 1);
}

float Butter1::process(float sample) {
  float out;

  out = cx * (sample + xd1) + cy * yd1;
  xd1 = sample; // x[n - 1] = x[n]
  yd1 = out; // y[n - 1] = y[n]

  return out;
}
