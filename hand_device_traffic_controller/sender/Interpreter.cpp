/*
Interpreter.cpp

Implementation file for Interpreter class.
Interpreter receives sensor data, analyze it, then return lane action commands.
*/

#include "Interpreter.h"
using namespace std;

//constructor
Interpreter::Interpreter() {
  lane = -1;
  action = -1;
  y = 0;
}

//sensor data input
void Interpreter::feed(float yaw, float pitch) {
  y = yaw;
  for (int i = 0; i < SIZE - 1; i++) {
    p[i] = p[i + 1];
  }
  p[SIZE - 1] = pitch;
  analyze();
}

//analyze data
void Interpreter::analyze() {
  computeLane();
  computeAction();
}

//use yaw value to determine lane value
void Interpreter::computeLane() {
  //reset lane
  lane = -1;
  //decide lane
  if (y < 45 && y >= -45) lane = 1; //top lane
  else if (y < 135 && y >= 45) lane = 2; //right lane
  else if (y < -135 || y >= 135) lane = 3; //bottom lane
  else if (y < -45 && y >= -135) lane = 4; //left lane
}

//use pitch value to determine action value
void Interpreter::computeAction() {
  //reset pinA
  action = -1;
  //check red and green signals: green override red
  if (isRed()) action = 0;
  if (isGreen()) action = 1;
}

//repackage lane value and action value into a 2 digit integer code for signal transmission over NRF module
int Interpreter::getLaneAction() {
  if (getLane() != -1 && getAction() != -1) return getLane() * 10 + getAction();
  else return -1;
}

//getter
int Interpreter::getLane() {
  return lane;
}

//getter
int Interpreter::getAction() {
  return action;
}

//logic to define red signal: following conditions must be true
//for past 2 seconds, hand device must be roughly pointing up vertically the whole time
bool Interpreter::isRed() {
  for (int i = 0; i < SIZE; i++) {
    if (p[i] < pitchUp - pitchOffSet || p[i] > pitchUp + pitchOffSet)
      return false;
  }
  return true;
}

//logic to define green signal: following conditions must be true
//for past 2 seconds, hand device's pitch must pass pitchUp 3 times, go below minimum threshold 3 times, and go above maximum threshold 3 times
bool Interpreter::isGreen() {
  int countlargeP = 0;
  int countSmallP = 0;
  int countUpP = 0;
  for (int i = 0; i < SIZE; i++) {
    if (p[i] < pitchUp - pitchOffSet)
      countSmallP++;
    if (p[i] > pitchUp + pitchOffSet)
      countlargeP++;
    if (p[i] > pitchUp - pitchOffSet && p[i] < pitchUp + pitchOffSet)
      countUpP++;
  }
  return (countSmallP > 2) && (countlargeP > 2) && (countUpP > 2);
}
