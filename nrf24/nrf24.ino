#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipe = 0xE8E8F0F0E1LL;
RF24 radio(9, 10);

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    char recievedMessage[32];
    radio.read(recievedMessage, 32);
    Serial.println(recievedMessage);
  }
}
