#ifndef INTERPRETER_H
#define INTERPRETER_H
#include <Adafruit_MPU6050.h>
#include "MemoryCell.h"

class Interpreter
{
private:
  bool greenPass;
  bool redStop;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  MemoryCell cell;
  void analyze();
public:
  Interpreter();
  void feed(sensors_event_t, sensors_event_t);
  bool passSignal();
  bool stopSignal();
};

#endif
