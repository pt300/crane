#ifndef PIDF_HEADER
#define PIDF_HEADER

#include "Filter.h"

/*
 * PIDF controller with intgral anti-windup
 */

class PIDF {
public:
  /*
   * Kp, Ki, Kd - Gain of proporitonal, integral and derivative terms of PIDF
   * saturation - Saturation for integral anti-windup
   * frequency - Sampling frequency
   */
  float Kp, Ki, Kd, saturation, frequency;
  Filter& filt;
  
  PIDF(float Kp, float Ki, float Kd, float saturation, Filter& filt, float frequency);

  /*
   * Resets memory to 0 and resets the filter
   */
  void reset();
  /*
   * Pass a sample through PIDF and return a result
   */
  float process(float sample);
private:
  /*
   * xd - previous sample
   * xdf - previous filtered sample, multiplied by frequency (used for derivative)
   */
  float xd, xdf; //x[n - 1] and filt{x[n - 1]}
  /*
   * Current integral value
   */
  float integral;
};

#endif
