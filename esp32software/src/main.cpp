#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "pair.h"

#define CE_PIN 4
#define CSN_PIN 5
#define SELF_ID 2

RF24 radio(CE_PIN, CSN_PIN);
PairContext pairCtx;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    SPI.begin(18, 19, 23, 5);

    if (!radio.begin())
    {
        Serial.println("radio.begin() failed");
        while (1) {}
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setCRCLength(RF24_CRC_16);

    pairingBegin(radio, pairCtx, SELF_ID);
}

void loop()
{
    pairingUpdate(radio, pairCtx);

    static bool printed = false;
    if (pairingIsComplete(pairCtx) && !printed)
    {
        printed = true;
        Serial.print("Paired with peer ");
        Serial.println(pairCtx.peerId);
    }

    delay(5);
}