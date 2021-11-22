#include "Interpreter.h"
using namespace std;

//contructor: initialize variables
Interpreter::Interpreter() {
  greenPass = false;
  redStop = false;
}

//read raw input sensors_event_t objects a and g, extract float values
void Interpreter::feed(sensors_event_t a, sensors_event_t g) {
  accel_x = a.acceleration.x;
  accel_y = a.acceleration.y;
  accel_z = a.acceleration.z;
  gyro_x = g.gyro.x;
  gyro_y = g.gyro.y;
  gyro_z = g.gyro.z;
  analyze();
}

//return current greenPass boolean value
bool Interpreter::passSignal() {
  return greenPass;
}

//return current redStop boolean value
bool Interpreter::stopSignal() {
  return redStop;
}

//analyze raw data to configure state values
void Interpreter::analyze() {
  if (accel_x > 9) {
    redStop = true;
  } else {
    redStop = false;
  }
  if (abs(gyro_y) > 2) {
    greenPass = true;
  } else {
    greenPass = false; 
  }
}
