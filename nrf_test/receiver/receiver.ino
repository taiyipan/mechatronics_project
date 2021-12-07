#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int n = 0;
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
  if (radio.available())              //Looking for the data.
  {
    int n=0;                 //Saving the incoming data  
    radio.read(&n, sizeof(n));    //Reading the data
    Serial.println(n);
  }
}
