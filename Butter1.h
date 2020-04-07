#ifndef BUTTER1_HEADER
#define BUTTER1_HEADER

#include "Filter.h"

/*
 * First order butterworth lowpass filter
 */

class Butter1 : public Filter {
public:
  Butter1(float frequency, float cutoff);
  virtual void reset() override;
  virtual float process(float sample) override;
private:
  float xd1, yd1; // x[n - 1] and y[n - 1]
  float cx, cy; //coefficients for x[n], x[n - 1] (cx) and y[n-1] (cy)
};

#endif
