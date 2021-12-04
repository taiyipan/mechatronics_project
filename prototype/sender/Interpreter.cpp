#include "Interpreter.h"
using namespace std;

Interpreter::Interpreter() {
  lane = -1;
  action = -1;
  y = 0;
}

void Interpreter::feed(float yaw, float pitch) {
  y = yaw;
  for (int i = 0; i < SIZE - 1; i++) {
    p[i] = p[i + 1];
  }
  p[SIZE - 1] = pitch;
  analyze();
}

void Interpreter::analyze() {
  computeLane();
  computeAction();
}

void Interpreter::computeLane() {
  //reset lane
  lane = -1;
  //decide lane
  if (y < 45 && y >= -45) lane = 0;
  else if (y < 135 && y >= 45) lane = 1;
  else if (y < -135 || y >= 135) lane = 2;
  else if (y < -45 && y >= -135) lane = 3;
}

void Interpreter::computeAction() {
  //reset pinA
  action = -1;
  //check red and green signals: green override red
  if (isRed()) action = 0;
  if (isGreen()) action = 1;
}

int Interpreter::getLaneAction() {
  return getLane() * 10 + getAction();
}

int Interpreter::getLane() {
  return lane;
}

int Interpreter::getAction() {
  return action;
}

bool Interpreter::isRed() {
  for (int i = 0; i < SIZE; i++) {
    if (p[i] < pitchUp - pitchOffSet || p[i] > pitchUp + pitchOffSet)
      return false;
  }
  return true;
}

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
  return (countSmallP > 3) && (countlargeP > 3) && (countUpP > 3);
}
