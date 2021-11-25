#include "MemoryCell.h"
using namespace std;

/*
Append sensor readings onto the end of memory arrays, and shift everything left, maintain total of 20 values
*/
void MemoryCell::load(float ax, float gy) {
  for (int i = 0; i < SIZE - 1; i++) {
    a[i] = a[i + 1];
    g[i] = g[i + 1];
  }
  a[SIZE - 1] = ax;
  g[SIZE - 1] = gy;
}

/*
If all of 20 elements in array a have values greater than 8, fire red signal
*/
bool MemoryCell::isRed() {
  for (int i = 0; i < SIZE; i++) {
    if (a[i] < toleranceA) {
      return false;
    }
  }
  return true;
}

/*
If within 20 elements of array g, values bigger than 2 and smaller than -2 are seen, and in array a, value greater than 8 is seen, fire green signal
*/
bool MemoryCell::isGreen() {
  int countSmallG = 0;
  int countLargeG = 0;
  int countLargeA = 0;
  for (int i = 0; i < SIZE; i++) {
    if (g[i] < -toleranceG)
      countSmallG++;
    if (g[i] > toleranceG)
      countLargeG++;
    if (a[i] > toleranceA)
      countLargeA++;
  }
  return (countSmallG > 3) && (countLargeG > 3) && (countLargeA > 3);
}
