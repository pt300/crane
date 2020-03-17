#ifndef PENDULUM_HEADER
#define PENDULUM_HEADER

#include <SPI.h>

class PENDULUM {
public:
  PENDULUM();
  /*
   * Initialise SPI to communicate with pendulum encoder
   * SS at pin 10
   */
  void begin();
  /*
   * Waits for pendulum to settle, then sets current encoder 
   * position as zero
   */
  void zero();
  /*
   * Reads current rotational position in radians
   * from -pi to pi
   */
  float read();
  /*
   * Returns raw positional data
   */
  int16_t read_raw();
private:
  const int ss;

  enum commands {READ = 0x00, RESET = 0x60, ZERO = 0x70};
  int16_t transaction(enum commands command);
};

#endif
