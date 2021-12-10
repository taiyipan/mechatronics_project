/*
Receiver code for Arduino Uno. Receives and interprets control signals from Sender. Manages traffic lights between automatic and manual mode. 
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//declarations
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";
bool manualMode = false;
bool vertical = true;
int n = -1; //integer variable to save data transmission from sender Arduino
int step = 0;
const int maxStep = 30; //value * 100ms traffic orientation cycle
bool bufferStart = false;
int buffer = 0;
const int maxBuffer = 30; //value * 100ms button buffer (during buffer, not responding to further button press; reduce sensitivity; prevent instability)
int yellowTimer = 0;
const int maxYellow = 10;

//declare button
const int toggleButton = 8;

//global constants
//A0 -> A5 map to pins 14 -> 19
//lane 1 (top)
const int lane1Red = 5;
const int lane1Yellow = 6;
const int lane1Green = 7;

//lane 2 (right)
const int lane2Red = 16; //A2
const int lane2Yellow = 15; //A1
const int lane2Green = 14; //A0

//lane 3 (bottom)
const int lane3Red = 2;
const int lane3Yellow = 3;
const int lane3Green = 4;

//lane 4 (left)
const int lane4Red = 19; //A5
const int lane4Yellow = 18; //A4
const int lane4Green = 17; //A3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // configure output pins
  pinMode(lane1Red, OUTPUT);
  pinMode(lane1Yellow, OUTPUT);
  pinMode(lane1Green, OUTPUT);
  pinMode(lane2Red, OUTPUT);
  pinMode(lane2Yellow, OUTPUT);
  pinMode(lane2Green, OUTPUT);
  pinMode(lane3Red, OUTPUT);
  pinMode(lane3Yellow, OUTPUT);
  pinMode(lane3Green, OUTPUT);
  pinMode(lane4Red, OUTPUT);
  pinMode(lane4Yellow, OUTPUT);
  pinMode(lane4Green, OUTPUT);

  //configure input pins
  pinMode(toggleButton, INPUT);

  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
}

void loop() {
  automaticMode();
  checkToggleButton();
  // put your main code here, to run repeatedly:
  if (radio.available()) { //Looking for the data
    radio.read(&n, sizeof(n)); //Reading the data
    interpret(n);
  }
  bufferIncrement();
  delay(100);
}

//in place of 999 remote singal, create localized 999 signal from local button press
void checkToggleButton() {
  if (digitalRead(toggleButton) == HIGH) interpret(999);
}

//allow button press to buffer so that repeated button press does not destablize code
void bufferIncrement() {
  if (bufferStart) buffer++; //increment buffer count
//  Serial.println(buffer);
  if (buffer >= maxBuffer) {
    //once buffer reaches threshold, reset buffer
    buffer = 0;
    bufferStart = false;
    }
}

//automatic operating mode
void automaticMode() {
  if (!manualMode) {
    if (vertical) verticalTraffic();
    else horizontalTraffic();
    step++; //increment step
    if (step >= maxStep) { //switch traffic orientation
      if (vertical) vertical = false;
      else vertical = true;
      step = 0;
      yellowTimer = 0;
    }
  }
}

//lane 1 and lane 3 green, other lanes red
void verticalTraffic() {
  //instant actions
  digitalWrite(lane1Red, LOW);
  digitalWrite(lane1Green, HIGH);
  digitalWrite(lane3Red, LOW);
  digitalWrite(lane3Green, HIGH);
  digitalWrite(lane2Green, LOW);
  digitalWrite(lane4Green, LOW);

  //timed actions
  if (yellowTimer < maxYellow) {
    digitalWrite(lane2Yellow, HIGH);
    digitalWrite(lane4Yellow, HIGH);
  } else {
    digitalWrite(lane2Yellow, LOW);
    digitalWrite(lane4Yellow, LOW);
    digitalWrite(lane2Red, HIGH);
    digitalWrite(lane4Red, HIGH);
  }
  yellowTimer++;
}

//lane 2 and lane 4 green, other lanes red
void horizontalTraffic() {
  //instant actions
  digitalWrite(lane2Red, LOW);
  digitalWrite(lane2Green, HIGH);
  digitalWrite(lane4Red, LOW);
  digitalWrite(lane4Green, HIGH);
  digitalWrite(lane1Green, LOW);
  digitalWrite(lane3Green, LOW);

  //timed actions
  if (yellowTimer < maxYellow) {
    digitalWrite(lane1Yellow, HIGH);
    digitalWrite(lane3Yellow, HIGH);
  } else {
    digitalWrite(lane1Yellow, LOW);
    digitalWrite(lane3Yellow, LOW);
    digitalWrite(lane1Red, HIGH);
    digitalWrite(lane3Red, HIGH);
  }
  yellowTimer++;
}

//interpret integer transmitted by sender Arduino
void interpret(int n) {
  if (n != -1) {
    Serial.println(n);
    if (n == 999) {
      if (buffer == 0) { //detect button press with buffer (prevent instability)
        Serial.println("Mode change detected");
        if (manualMode) manualMode = false;
        else manualMode = true;
        bufferStart = true; //start buffer count
        }
      }
    else {
      if (manualMode) execute(n);
    }
    n = -1; //reset n
  }
}

//execute commands transmitted from sender Arduino
void execute(int command) {
  //break command into lane and action
  int lane = command / 10;
  int action = command % 10;
  //execute accordingly
  switch(lane) {
    case 1: //lane 1 (front 1, then clockwise increment)
      if (action == 0) { //red signal
        digitalWrite(lane1Green, LOW);
        digitalWrite(lane1Yellow, LOW);
        digitalWrite(lane1Red, HIGH);
      } else if (action == 1) { //green signal
        digitalWrite(lane1Red, LOW);
        digitalWrite(lane1Yellow, LOW);
        digitalWrite(lane1Green, HIGH);
      }
      break;
    case 2: //lane 2
      if (action == 0) { //red signal
        digitalWrite(lane2Green, LOW);
        digitalWrite(lane2Yellow, LOW);
        digitalWrite(lane2Red, HIGH);
      } else if (action == 1) { //green signal
        digitalWrite(lane2Red, LOW);
        digitalWrite(lane2Yellow, LOW);
        digitalWrite(lane2Green, HIGH);
      }
      break;
    case 3: //lane 3
      if (action == 0) { //red signal
        digitalWrite(lane3Green, LOW);
        digitalWrite(lane3Yellow, LOW);
        digitalWrite(lane3Red, HIGH);
      } else if (action == 1) { //green signal
        digitalWrite(lane3Red, LOW);
        digitalWrite(lane3Yellow, LOW);
        digitalWrite(lane3Green, HIGH);
      }
      break;
    case 4: //lane 4
      if (action == 0) { //red signal
        digitalWrite(lane4Green, LOW);
        digitalWrite(lane4Yellow, LOW);
        digitalWrite(lane4Red, HIGH);
      } else if (action == 1) { //green signal
        digitalWrite(lane4Red, LOW);
        digitalWrite(lane4Yellow, LOW);
        digitalWrite(lane4Green, HIGH);
      }
      break;
  }
}
