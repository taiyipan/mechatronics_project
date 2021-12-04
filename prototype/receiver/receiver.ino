#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <String.h>
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
}

void loop() {
  // put your main code here, to run repeatedly:
  if (radio.available()) { //Looking for the data
    radio.read(&n, sizeof(n)); //Reading the data
  }
}

//execute commands given by interpreter object
void execute(int command) {
  //break command into lane and action
  int lane = command / 10;
  int action = command % 10;
  //execute accordingly
  switch(lane) {
    case 0: //lane 0 (front 0, then clockwise increment)
      if (action == 0) { //red signal
        digitalWrite(5, HIGH);
        digitalWrite(6, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(5, LOW);
        digitalWrite(6, HIGH);
      }
      break;
    case 1: //lane 1
      if (action == 0) { //red signal
        digitalWrite(9, HIGH);
        digitalWrite(10, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(9, LOW);
        digitalWrite(10, HIGH);
      }
      break;
    case 2: //lane 2
      if (action == 0) { //red signal
        digitalWrite(3, HIGH);
        digitalWrite(4, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(3, LOW);
        digitalWrite(4, HIGH);
      }
      break;
    case 3: //lane 3
      if (action == 0) { //red signal
        digitalWrite(7, HIGH);
        digitalWrite(8, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(7, LOW);
        digitalWrite(8, HIGH);
      }
      break;
  }
}
