#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "pair.h"

#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);

// CHANGE THIS ON EACH BOARD
static const NodeRole THIS_ROLE = ROLE_B;
// other board: static const NodeRole THIS_ROLE = ROLE_B;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    SPI.begin(18, 19, 23, 5);

    if (!radio.begin())
    {
        Serial.println("Radio hardware failed");
        while (1) {}
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setCRCLength(RF24_CRC_16);

    Serial.println("Radio ready");

    if (runPairing(radio, THIS_ROLE))
    {
        Serial.println("Devices paired successfully!");
    }
    else
    {
        Serial.println("Pairing failed");
    }
}

void loop()
{
}