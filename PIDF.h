#ifndef PIDF_HEADER
#define PIDF_HEADER

#include "Filter.h"

class PIDF {
public:
  float Kp, Ki, Kd, saturation, frequency;
  Filter& filt;
  
  PIDF(float Kp, float Ki, float Kd, float saturation, Filter& filt, float frequency);

  void reset();
  float process(float sample);
private:
  float xd, xdf; //x[n - 1] and filt{x[n - 1]}
  float integral;
};

#endif
