#include "CART.h"
#include "MOTOR.h"

MOTOR Motor;
CART Cart(Motor);

void setup() {
  Serial.begin(115200);
  Cart.begin();
  Motor.begin();
 
  Serial.println("Starting");
  delay(2000);
  Cart.calibrate(0.5, 0.8);
  delay(2000);
  Cart.move_to(0, 50);
}

void loop() {
  int32_t pos;
  float v;
  float in;

  in = Cart.read();
  Serial.println(in);

  
/*  for(;;) {
    pos = Cart.read_raw();
    v = pos;
    v = v * -0.01;
    Motor.write_voltage(v);
    SerialUSB.write((char *)&v, 4);
  }*/
}
