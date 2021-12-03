#include <QMC5883LCompass.h>
QMC5883LCompass compass;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  compass.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  int x, y, z, a, b;
  char myArray[3];
  
  compass.read();
  
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  
  a = compass.getAzimuth();
  
  b = compass.getBearing(a);

  compass.getDirection(myArray, a);
  
  
  Serial.print("X: ");
  Serial.print(x);

  Serial.print(" Y: ");
  Serial.print(y);

  Serial.print(" Z: ");
  Serial.print(z);

  Serial.print(" Azimuth: ");
  Serial.print(a);

  Serial.print(" Bearing: ");
  Serial.print(b);

  Serial.print(" Direction: ");
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);

  Serial.println();

  delay(100);
}
