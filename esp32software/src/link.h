#pragma once

#include <Arduino.h>
#include <RF24.h>

enum LinkRole
{
    LINK_ROLE_POLLER,
    LINK_ROLE_RESPONDER
};

// nRF24 max payload is 32 bytes.
// Layout:
//  1  deviceId
//  1  flags
//  2  seq
//  4  timeUs
// 12  posX,posY,posZ
//  4  yawDeg
//  4  initYawDeg
//  4  speedMps
// ----------------
// 32 bytes total
struct __attribute__((packed)) StatePacket
{
    uint8_t  deviceId;
    uint8_t  flags;
    uint16_t seq;
    uint32_t timeUs;

    float posX;
    float posY;
    float posZ;

    float yawDeg;       // current yaw
    float initYawDeg;   // yaw captured at boot for shared-frame alignment
    float speedMps;     // scalar speed magnitude
};

static_assert(sizeof(StatePacket) == 32, "StatePacket must be 32 bytes");

void linkBegin(RF24 &radio, LinkRole role, uint8_t selfId);
void linkSetLocalState(const StatePacket &pkt);
bool linkExchange(RF24 &radio, StatePacket &peerPkt);
bool linkPollResponder(RF24 &radio, StatePacket &rxPkt);