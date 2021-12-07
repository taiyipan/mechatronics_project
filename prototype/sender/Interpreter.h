#ifndef INTERPRETER_H
#define INTERPRETER_H

class Interpreter
{
private:
  const static int SIZE = 15;
  const float pitchUp = 85;
  const float pitchOffSet = 30;
  float y; //yaw value, real time
  float p[SIZE]; //pitch array, last 2 seconds, in 20 timesteps, 100ms each timestep
  int lane, action;
  void analyze();
  bool isRed();
  bool isGreen();
  void computeLane();
  void computeAction();
public:
  Interpreter(); //initialize pinN pinA to -1 null state
  void feed(float, float); //obtain yaw and pitch values from driver file
  int getLaneAction();
  int getLane(); //return lane designation; return -1 if null
  int getAction(); //return lane action: 0 is red, 1 is green; return -1 if null
};

#endif
