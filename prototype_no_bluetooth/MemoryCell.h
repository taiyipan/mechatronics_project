#ifndef MEMORYCELL_H
#define MEMORYCELL_H

class MemoryCell
{
private:
  const static int SIZE = 20;
  const float toleranceA = 8;
  const float toleranceG = 2;
  float a[SIZE]; //last 2 seconds, in 20 timesteps, 100ms each timestep
  float g[SIZE]; //last 2 seconds, in 20 timesteps, 100ms each timestep
public:
  void load(float, float);
  bool isRed();
  bool isGreen();
};

#endif
