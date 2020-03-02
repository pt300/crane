#ifndef CART_HEADER
#define CART_HEADER

#include "MOTOR.h"

class CART {
public:
  CART(MOTOR& motor);
  /*
   * Sets up Timer/Counter 2 to quadrature demodulation mode
   * PHA on pin 5
   * PHB on pin 4
   * 
   * Left switch pin 6
   * Right switch pin 7
   */
  void begin();
  /*
   * Reads current value from counter
   */
  int32_t read_raw();
  /*
   * Reads current position realtive to calibration data
   * Output between -1 and 1
   * Do not use without performing a calibration first!
   */
  float read();
  /*
   * Performs calibration by moving cart between ends of rail
   * Optional parameters specify voltage fractions to use when
   * quicly moving from to each end and slowly ensure precise 
   * positioning on switch. Low enough to not ram into end, low
   * enough to overcome static friction.
   */
  void calibrate(float slow = 0.15, float fast = 0.30);
  /*
   * Tries to move cart to requested position
   * uses proportional controller with optional given Kp
   * position from -1 to 1
   * Stops when error <= 0.001 or edge switch is hit
   */
  void move_to(float position, float P = 30);
private:
  MOTOR& motor;
  int right_side, left_side;
  enum side {LEFT, RIGHT};
  void reach(float slow, float fast, enum side side);
};

#endif
