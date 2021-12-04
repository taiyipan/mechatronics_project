#include <SPI.h>
#include <String.h>
#include <nRF24L01.h>
#include <RF24.h>
int a=0;
RF24 radio(9, 10); // CE, CSN         
const byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.
int button_pin = 2;
boolean button_state = 0;
void setup() {
pinMode(button_pin, INPUT);
Serial.begin(9600);
radio.begin();                  //Starting the Wireless communication
radio.openWritingPipe(address); //Setting the address where we will send the data
radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
radio.stopListening();  //This sets the module as transmitter

}
void loop()
{
  if(true)
  {
    //String s=Serial.readStringUntil('\n');
  // char s=Serial.read();
    String s = String(Serial.read(),3);
    a++;
    Serial.print(a);
    radio.write(&a, sizeof(a)); 
    delay(1000);
  }

}
