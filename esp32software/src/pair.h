#pragma once
#include <Arduino.h>
#include <RF24.h>

// Optional discovery/pairing helper for assigning the two fixed node IDs.
// The current main sketch hardcodes IDs and does not call this module, but the
// interface remains useful for testing alternate bring-up flows.

struct PairContext
{
    bool paired = false;
    uint8_t selfId = 0;
    uint8_t peerId = 0;
};

void pairingBegin(RF24 &radio, PairContext &ctx, uint8_t selfId);
void pairingUpdate(RF24 &radio, PairContext &ctx);
bool pairingIsComplete(const PairContext &ctx);
