#ifndef CART_HEADER
#define CART_HEADER

#include "MOTOR.h"

class CART {
public:
  CART(MOTOR& motor, float length = 2.0);
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
   * Reads current position realative to center of rail
   * Output between -length/2 and length/2
   * Do not use without performing a calibration first!
   */
  float read();
  /*
   * Performs calibration by moving cart between ends of rail
   * Optional parameters specify voltage to use when
   * quicly moving from to each end and slowly ensure precise 
   * positioning on switch. Low enough to not ram into end, high
   * enough to overcome static friction.
   */
  void calibrate(float slow = 3.5, float fast = 5.5);
  /*
   * Tries to move cart to requested position
   * uses proportional controller with optional given Kp
   * position from -rail_length/2 to rail_length/2
   * Stops when error <= 0.01 or edge switch is hit
   * 
   * Return true if edge was hit during the move
   */
  bool move_to(float position, float P = 550.0);
  /*
   * Check if cart has hit edge switch
   */
  bool edge_hit();
  /*
   * Half of length of rail in meters
   */
  float rail_length;
  /*
   * Half of rail limit for move_to
   */
  float rail_limit;
private:
  MOTOR& motor;
  int32_t left_side, right_side;
  enum side {LEFT, RIGHT};
  void reach(float slow, float fast, enum side side);
};

#endif
