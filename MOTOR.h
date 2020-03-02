#ifndef MOTOR_HEADER
#define MOTOR_HEADER

class MOTOR {
public:
  MOTOR();
  /*
   * Sets up channel 0 of TC0 to PWM mode. 15kHz
   * DIR on pin 2
   * OUT on pin 3
   */
  void begin();
  /*
   * Sets current duty cycle. Maps 0-2800 to 0-100%
   */
  void write_duty(int duty);
  /*
   * Sets current voltage provided to DC motor.
   * Takes voltage fraction from -1 to 1
   */
  void write_voltage(float voltage);
};

#endif
