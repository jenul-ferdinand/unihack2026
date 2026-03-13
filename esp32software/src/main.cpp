#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define CE_PIN  4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "NODE1";

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!radio.begin()) {
    Serial.println("Radio failed to start");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);

  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("Receiver ready...");
}

void loop() {
  if (radio.available()) {

    char receivedText[32] = {0};

    radio.read(&receivedText, sizeof(receivedText));

    Serial.print("Received string: ");
    Serial.println(receivedText);
  }
}