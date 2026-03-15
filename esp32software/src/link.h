#pragma once

#include <Arduino.h>
#include <RF24.h>

// Lightweight transport on top of nRF24 auto-ack payloads.
//
// The poller transmits its state as the main payload and receives the peer's
// state in the auto-ack payload. The responder mirrors that pattern by always
// queuing its current state as the next ack payload.

enum LinkRole
{
    LINK_ROLE_POLLER,
    LINK_ROLE_RESPONDER
};

enum LinkPeerDirection : uint8_t
{
    LINK_PEER_UNKNOWN = 0,
    LINK_PEER_FORWARD = 1,
    LINK_PEER_RIGHT = 2,
    LINK_PEER_BACK = 3,
    LINK_PEER_LEFT = 4,
    LINK_PEER_CLOSE = 5
};

static constexpr uint16_t LINK_SEQ_COUNTER_MASK = 0x07FF;

inline uint16_t linkPackSeq(uint16_t counter, uint8_t peerDirection, uint8_t confidence)
{
    return static_cast<uint16_t>(
        (counter & LINK_SEQ_COUNTER_MASK) |
        ((peerDirection & 0x7u) << 11) |
        ((confidence & 0x3u) << 14));
}

inline uint16_t linkUnpackCounter(uint16_t packedSeq)
{
    return packedSeq & LINK_SEQ_COUNTER_MASK;
}

inline uint8_t linkUnpackPeerDirection(uint16_t packedSeq)
{
    return static_cast<uint8_t>((packedSeq >> 11) & 0x7u);
}

inline uint8_t linkUnpackPeerConfidence(uint16_t packedSeq)
{
    return static_cast<uint8_t>((packedSeq >> 14) & 0x3u);
}

// Compact state packet sized to fit exactly in one nRF24 payload.
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

// Poller-side exchange: send local state and read the responder's ack payload.
bool linkExchange(RF24 &radio, StatePacket &peerPkt);

// Responder-side update: consume a poll packet and queue the next ack payload.
bool linkPollResponder(RF24 &radio, StatePacket &rxPkt);
