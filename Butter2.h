#ifndef BUTTER2_HEADER
#define BUTTER2_HEADER

#include "Filter.h"

/*
 * Second order butterworth lowpass filter
 */

class Butter2 : public Filter {
public:
  Butter2(float frequency, float cutoff);
  virtual void reset() override;
  virtual float process(float sample) override;
private:
  float xd1, xd2, yd1, yd2; // x[n - 1] and y[n - 1]
  float cx02, cx1, cy1, cy2; //coefficients for x[n], x[n-2] (cx02), x[n-1] (cx1), y[n-1] (cy1), y[n-2] (cy2)
};

#endif
