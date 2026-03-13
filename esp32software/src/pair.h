#pragma once
#include <Arduino.h>
#include <RF24.h>

struct PairContext
{
    bool paired = false;
    uint8_t selfId = 0;
    uint8_t peerId = 0;
};

void pairingBegin(RF24 &radio, PairContext &ctx, uint8_t selfId);
void pairingUpdate(RF24 &radio, PairContext &ctx);
bool pairingIsComplete(const PairContext &ctx);