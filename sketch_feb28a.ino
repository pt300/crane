#include <DueTimer.h>
#include <RingBuf.h>
#include <AsyncDelay.h>

#include "MOTOR.h"
#include "PENDULUM.h"
#include "CART.h"
#include "VarKV.h"
#include "PIDF.h"
#include "Butter1.h"

#define FSAMP 100

MOTOR Motor(20.0);
CART Cart(Motor, 1.14);
PENDULUM Pendulum;


#define VAR(a) {#a, &a}

/// !!! Modify code here !!!
// Changeable variables
float pendulum_length = 0.25;

Butter1 filt1(5, FSAMP);
PIDF pid1(1, 0, 0, 20, filt1, FSAMP);

float Pp = 100;
float Ip = 0;
float Dp = 0.5;
float Ap = 0;
  
float Pc = 0;
float Ic = 0;
float Dc = 0;
float Ac = 0;

VarKV vars({
  VAR(pendulum_length),
  VAR(Pp),VAR(Ip),VAR(Dp),VAR(Ap),
  VAR(Pc),VAR(Ic),VAR(Dc),VAR(Ac),
  {"Kp_pendulum", &pid1.Kp}, {"Ki_pendulum", &pid1.Ki}, {"Kd_pendulum", &pid1.Kd}, {"Cf_pendulum", &pid1.filt.cutoff} 
  });

/// Controller
/*
 * angle: of pendulum in radians from resting position
 * position: of the load on the rail in meters
 * target: current position target
 * period: controller period
 * 
 * returns: voltage input to motor
 */
float controller(float angle, float position, float target, float frequency) {
  static float Ep_int = 0;
  static float Ep_1 = 0;
  static float Ec_int = 0;
  static float Ec_1 = 0;

//  angle += PI;
//  if(angle > PI)
//    angle -= 2*PI;
 
  float Ep = -angle;
  float Ec = target - position;

  Ep_int = Ep_int + Ep/frequency;
  Ec_int = Ec_int + Ec/frequency;
  
  float Cp = Ep * Pp + Ip*(Ep_int - Ap*0) + Dp*(Ep-Ep_1)*frequency;
  float Cc = Ec * Pc + Ic*(Ec_int - Ac*0) + Dc*(Ec-Ec_1)*frequency;

  Ep_1 = Ep;
  Ec_1 = Ec;
  
  return Cc + Cp; 
}
///

struct Sample {
  uint32_t time;
  float angle, position, voltage;
} __attribute__((packed));

volatile float target = 0;
volatile bool controller_enabled = false;
volatile bool stream_enabled = false;

RingBuf<Sample, 50> buf;

void call_controller() {
  float voltage = 0;
//  float f = 0.1; //float f = 1.0954;
  float angle = Pendulum.read();
  float position = Cart.read();  
  
  if(abs(position) > Cart.rail_limit) { //add switch interrupt?
    controller_enabled = false;
    Motor.write_voltage(0);
  }

  //target = sin( millis() / 1000.0 * f * 2 * PI) * 0.4;
  voltage = controller(angle, position - pendulum_length * sin(angle), target, Timer2.getFrequency());
  
  if(stream_enabled)
    buf.push({micros(), angle, position, voltage});
  
  if(controller_enabled == false)
    return;
  
  Motor.write_voltage(voltage);
}

#define SER SerialUSB

char *send_buf;
int send_buf_size;
int send_buf_elem_size;
int send_buf_index;

AsyncDelay send_timeout(20, AsyncDelay::MILLIS);


void setup() {
  SER.begin(115200);
  SER.setTimeout(100);
  Cart.begin();
  Motor.begin();
  Pendulum.begin();

  send_buf_elem_size = sizeof (Sample) + 3;
  send_buf_size = SER.availableForWrite() / send_buf_elem_size;
//  assert(send_buf_size > 1);
  send_buf = new char[send_buf_elem_size * send_buf_size];
  send_buf_index = 0;
 
  delay(2000);
  Cart.calibrate();

  Timer2.stop().attachInterrupt(call_controller).setFrequency(FSAMP);

  while(SER.available()) //remove anything received during long calibration
    SER.read();
}

#define GETFLOAT(a) if(SER.readBytes((char *)&a, 4) != 4) \
                      break;;
#define RESPOND if(!stream_enabled) SER.write("RES");

#define GETCHAR(a) a = 0; \
                   a = SER.readBytes((char *)&a, 1) == 0 ? -1 : a;

void loop() {
  Sample samp;

  // Load send buffer with new samples as long as there's space and samples
  while(buf.pop(samp) && send_buf_index != send_buf_size) {
    //char head[] = {'S', 'T','R'};
    memcpy(&send_buf[send_buf_index], "STR", 3);
    send_buf_index += 3;
    memcpy(&send_buf[send_buf_index], &samp, sizeof samp);
    send_buf_index += sizeof samp;
//    SER.write("STR");
//    SER.write((char *)&samp, sizeof samp);
  }

  // if buffer is full or time since last samples was too long send the buffer
  // this part can lock if the previous buffer is still being sent, will result
  // in samples being dropped on ring buffer
  if((send_buf_index == (send_buf_size * send_buf_elem_size) ||
      (send_timeout.isExpired() && send_buf_index != 0)) &&
      stream_enabled) {
    int bytes = send_buf_index;
    send_buf_index = 0;

    SER.write((char *) send_buf, bytes);
    send_timeout.restart();
  }
  
  if(SER.available() == 0)
    return;
  
  char req_str[3] = {0, 0, 0};
  SER.readBytes(req_str, 3);
  if(memcmp(req_str, "REQ", 3) != 0)
    return;

  int c;
  GETCHAR(c);
  switch(c) {
    //recalibrate
    /*case 'R':
      Timer2.stop();
      Motor.write_voltage(0);
      Cart.calibrate();
      RESPOND;
      break;*/
    //controller
    case 'C':
      GETCHAR(c);
      if(c == 'B') {
        controller_enabled = true;
        RESPOND;
      }
      else if(c == 'E') {
        controller_enabled = false;
        Motor.write_voltage(0);
        RESPOND;
      }
      break;
    //move
    case 'M':
      float pos;
      GETFLOAT(pos);
      Cart.move_to(pos);
      RESPOND;
      break;
    //voltage
    case 'V':
      GETCHAR(c);
      float voltage;
      if(c == 'S') {
        GETFLOAT(voltage);
        Motor.write_voltage(voltage);
        RESPOND;
      }
      else if(c == 'L') {
        GETFLOAT(voltage);
        Motor.voltage_limit = voltage;
        RESPOND;
      }
      break;
    //stream
    case 'S':
      GETCHAR(c);
      if(c == 'B') {
        RESPOND;
        stream_enabled = true;
      }
      else if(c == 'E') {
        stream_enabled = false;
        buf.clear();
        RESPOND;
      }
      break;
    //target
    case 'T':
      float target_new;
      GETFLOAT(target_new);
      target = constrain(target_new, -Cart.rail_limit, Cart.rail_limit);
      RESPOND;
      break;
    //sampling freq
    case 'F':
      float freq;
      GETFLOAT(freq);
      Timer2.stop().setFrequency(constrain(freq, 1, 5000)).start();
      RESPOND;
      break;
    //ping
    case 'P':
      RESPOND;
      break;
    case 'K':
      GETCHAR(c);
      if(c == 'S') {
        char name[51];
        int len;
        float value;

        GETCHAR(len);
        if(len > 50 || len < 0)
          break;
          
        if(SER.readBytes(name, len) != len)
          break;
        name[len] = 0;

        GETFLOAT(value);
        
        String tmp(name);
        if(!vars.update(tmp, value))
          break;
        RESPOND;
      }
      else if(c == 'L' && !stream_enabled) {
        RESPOND;
        byte size = vars.size;
        SER.write(&size, 1);
        for(int i = 0; i < vars.size; i++) {
          String &cur = vars.keys[i];
          byte cur_size = cur.length();
          SER.write((char *)&cur_size, 1);
          SER.write(cur.c_str(), cur_size);
          SER.write((char *)vars.values[i], 4);
        }
      }
      break;
    case 'Q':
      GETCHAR(c);
      if(c == 'C') {
        RESPOND;
        SER.write((char *)&Cart.rail_length, 4);
      }
      else if(c == 'V')   {
        RESPOND;
        SER.write((char *)&Motor.voltage_max, 4);
      }
    default:
    //ignore
      break;
  }
}

/*
 * Interface:
 * packet:
 * "REQ",command,params
 * "RES"
 * "STR",pendulum(float), cart(float)
 * commands:
 * CB(), start controller
 * CE(), stop controller
 * R(), force recallibration of rail width, stops controller-
 * M(float pos), move cart to `pos` on rail, in meters, 0 in center
 * VS(float v), set motor voltage input to `v` volts
 * VL(float v), limit motor voltage to <-v; v>
 * SB(), start streaming pendulum and cart data
 * SE(), end data stream
 * T(float target), set current target, in m, limited to 80% on each side 
 * F(float freq), set sampling frequency
 * P(), ping-pong
 * KS(byte str_len, char[str_len] name, float value), set variable `name` to value `value`
 * KL(), list variables; response: RES, num_values, [str_len, name, value_single]
 * QC(), get programmed half rail length
 * QV(), get programmed max voltage
 */
