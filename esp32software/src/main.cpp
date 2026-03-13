#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "imu.h"
#include "link.h"

#define CE_PIN 4
#define CSN_PIN 5

// CHANGE THESE PER DEVICE
#define SELF_ID   2
#define IS_POLLER 0

RF24 radio(CE_PIN, CSN_PIN);

static uint16_t gSeq = 0;
static StatePacket localPkt = {};
static StatePacket peerPkt  = {};

static uint8_t buildFlags()
{
    // Fill these in later when you add real detectors:
    // bit0 = stance
    // bit1 = ZUPT active
    // bit2 = stationary
    // bit3 = heading reliable
    // bit4 = close-contact event
    // bit5 = range-valid hint
    return 0;
}

static void fillLocalPacket(StatePacket &pkt)
{
    float px, py, pz;
    float vx, vy, vz;

    imu_getPosition(px, py, pz);
    imu_getVelocity(vx, vy, vz);

    pkt.deviceId = SELF_ID;
    pkt.flags    = buildFlags();
    pkt.seq      = gSeq++;
    pkt.timeUs   = micros();

    pkt.posX = px;
    pkt.posY = py;
    pkt.posZ = pz;

    pkt.velX = vx;
    pkt.velY = vy;
    pkt.velZ = vz;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    SPI.begin(18, 19, 23, 5);

    if (!imu_begin())
    {
        Serial.println("imu_begin() failed");
        while (1) {}
    }

    if (!radio.begin())
    {
        Serial.println("radio.begin() failed");
        while (1) {}
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setCRCLength(RF24_CRC_16);

#if IS_POLLER
    linkBegin(radio, LINK_ROLE_POLLER, SELF_ID);
    Serial.print("Started as POLLER, SELF_ID=");
    Serial.println(SELF_ID);
#else
    linkBegin(radio, LINK_ROLE_RESPONDER, SELF_ID);
    Serial.print("Started as RESPONDER, SELF_ID=");
    Serial.println(SELF_ID);
#endif
}

void loop()
{
    imu_update();

    fillLocalPacket(localPkt);
    linkSetLocalState(localPkt);

#if IS_POLLER
    if (linkExchange(radio, peerPkt))
    {
        Serial.print("RX peer=");
        Serial.print(peerPkt.deviceId);
        Serial.print(" seq=");
        Serial.print(peerPkt.seq);
        Serial.print(" t=");
        Serial.print(peerPkt.timeUs);
        Serial.print(" pos=(");
        Serial.print(peerPkt.posX, 3);
        Serial.print(", ");
        Serial.print(peerPkt.posY, 3);
        Serial.print(", ");
        Serial.print(peerPkt.posZ, 3);
        Serial.print(") vel=(");
        Serial.print(peerPkt.velX, 3);
        Serial.print(", ");
        Serial.print(peerPkt.velY, 3);
        Serial.print(", ");
        Serial.print(peerPkt.velZ, 3);
        Serial.println(")");
    }
    else
    {
        Serial.println("Exchange failed");
    }

    delay(20); // 50 Hz poll rate
#else
    if (linkPollResponder(radio, peerPkt))
    {
        Serial.print("RX peer=");
        Serial.print(peerPkt.deviceId);
        Serial.print(" seq=");
        Serial.print(peerPkt.seq);
        Serial.print(" t=");
        Serial.print(peerPkt.timeUs);
        Serial.print(" pos=(");
        Serial.print(peerPkt.posX, 3);
        Serial.print(", ");
        Serial.print(peerPkt.posY, 3);
        Serial.print(", ");
        Serial.print(peerPkt.posZ, 3);
        Serial.print(") vel=(");
        Serial.print(peerPkt.velX, 3);
        Serial.print(", ");
        Serial.print(peerPkt.velY, 3);
        Serial.print(", ");
        Serial.print(peerPkt.velZ, 3);
        Serial.println(")");
    }

    delay(2);
#endif
}