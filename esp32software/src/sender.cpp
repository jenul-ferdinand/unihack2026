#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define CE_PIN  4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "NODE1";

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("Radio failed");
    while (1);
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void loop() {
  const char text[] = "Hello World";

  bool success = radio.write(&text, sizeof(text));

  if (success) {
    Serial.println("Sent: Hello World");
  } else {
    Serial.println("Send failed");
  }

  delay(1000);
}