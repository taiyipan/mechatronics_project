#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int a = 0;
int b = 1;
int i = 0;
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();  //This sets the module as transmitter
}

void loop() {
  // put your main code here, to run repeatedly:
  if (i % 2 == 0) { //even 
    Serial.println(a);
    radio.write(&a, sizeof(a)); 
  } else { //odd
    Serial.println(b);
    radio.write(&a, sizeof(b)); 
  }
  i++;
  delay(1000);
}
