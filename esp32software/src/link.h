#pragma once
#include <Arduino.h>
#include <RF24.h>

enum LinkRole
{
    LINK_ROLE_POLLER,
    LINK_ROLE_RESPONDER
};

struct __attribute__((packed)) StatePacket
{
    uint8_t  deviceId;
    uint8_t  flags;
    uint16_t seq;
    uint32_t timeUs;
    float    posX;
    float    posY;
    float    posZ;
    float    velX;
    float    velY;
    float    velZ;
};

static_assert(sizeof(StatePacket) == 32, "StatePacket must be 32 bytes");

void linkBegin(RF24 &radio, LinkRole role, uint8_t selfId);
void linkSetLocalState(const StatePacket &pkt);
bool linkExchange(RF24 &radio, StatePacket &peerPkt);
bool linkPollResponder(RF24 &radio, StatePacket &rxPkt);