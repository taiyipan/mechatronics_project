#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <String.h>
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";
bool manualMode = false;
bool vertical = true;
int n = 0;
int step = 0;
const int maxStep = 30;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // configure output pin
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
}

void loop() {
  automaticMode();
  // put your main code here, to run repeatedly:
  if (radio.available()) { //Looking for the data
    radio.read(&n, sizeof(n)); //Reading the data
  }
  interpret(n);
  delay(100);
}

void automaticMode() {
  if (!manualMode) {
    if (vertical) verticalTraffic();
    else horizontalTraffic();
    step++; //increment step
    if (step >= maxStep) { //switch traffic orientation
      if (vertical) vertical = false;
      else vertical = true;
      step = 0;
    }
  }
}

//lane 0 and lane 2 green, other lanes red
void verticalTraffic() {
  execute(01);
  execute(21);
  execute(10);
  execute(30);
}

//lane 1 and lane 3 green, other lanes red
void horizontalTraffic() {
  execute(00);
  execute(20);
  execute(11);
  execute(31);
}

void interpret(int n) {
  if (n != 0) {
    if (n == 999) {
      if (manualMode) manualMode = false;
      else manualMode = true;
    }
    else execute(n);
    //reset n
    n = 0;
  }
}

//execute commands transmitted from sender Arduino
void execute(int command) {
  //break command into lane and action
  int lane = command / 10;
  int action = command % 10;
  //execute accordingly
  switch(lane) {
    case 0: //lane 0 (front 0, then clockwise increment)
      if (action == 0) { //red signal
        digitalWrite(2, HIGH);
        digitalWrite(3, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(2, LOW);
        digitalWrite(3, HIGH);
      }
      break;
    case 1: //lane 1
      if (action == 0) { //red signal
        digitalWrite(6, HIGH);
        digitalWrite(7, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(6, LOW);
        digitalWrite(7, HIGH);
      }
      break;
    case 2: //lane 2
      if (action == 0) { //red signal
        digitalWrite(0, HIGH);
        digitalWrite(1, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(0, LOW);
        digitalWrite(1, HIGH);
      }
      break;
    case 3: //lane 3
      if (action == 0) { //red signal
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);
      } else if (action == 1) { //green signal
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
      }
      break;
  }
}
