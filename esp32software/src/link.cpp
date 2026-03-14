#include "link.h"
#include <string.h>

static const byte ADDR_NODE1[6] = "NODE1";
static const byte ADDR_NODE2[6] = "NODE2";

static LinkRole gRole;
static uint8_t gSelfId = 0;
static StatePacket gLocalState = {};
static bool gHaveLocalState = false;

static const byte *selfAddr()
{
    return (gSelfId == 1) ? ADDR_NODE1 : ADDR_NODE2;
}

static const byte *peerAddr()
{
    return (gSelfId == 1) ? ADDR_NODE2 : ADDR_NODE1;
}

void linkBegin(RF24 &radio, LinkRole role, uint8_t selfId)
{
    gRole = role;
    gSelfId = selfId;
    gHaveLocalState = false;
    memset(&gLocalState, 0, sizeof(gLocalState));

    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.setRetries(5, 15);

    radio.flush_tx();
    radio.flush_rx();

    radio.openWritingPipe(peerAddr());
    radio.openReadingPipe(1, selfAddr());

    if (gRole == LINK_ROLE_POLLER)
    {
        radio.stopListening();
    }
    else
    {
        radio.startListening();

        StatePacket emptyAck = {};
        emptyAck.deviceId = gSelfId;
        radio.writeAckPayload(1, &emptyAck, sizeof(emptyAck));
    }
}

void linkSetLocalState(const StatePacket &pkt)
{
    gLocalState = pkt;
    gHaveLocalState = true;
}

bool linkExchange(RF24 &radio, StatePacket &peerPkt)
{
    if (gRole != LINK_ROLE_POLLER || !gHaveLocalState)
        return false;

    radio.stopListening();

    const bool ok = radio.write(&gLocalState, sizeof(gLocalState));
    if (!ok)
        return false;

    if (!radio.isAckPayloadAvailable())
        return false;

    const uint8_t len = radio.getDynamicPayloadSize();
    if (len != sizeof(StatePacket))
    {
        radio.flush_rx();
        return false;
    }

    radio.read(&peerPkt, sizeof(peerPkt));
    return true;
}

bool linkPollResponder(RF24 &radio, StatePacket &rxPkt)
{
    if (gRole != LINK_ROLE_RESPONDER)
        return false;

    if (!radio.available())
        return false;

    const uint8_t len = radio.getDynamicPayloadSize();
    if (len != sizeof(StatePacket))
    {
        radio.flush_rx();
        return false;
    }

    radio.read(&rxPkt, sizeof(rxPkt));

    if (gHaveLocalState)
    {
        radio.writeAckPayload(1, &gLocalState, sizeof(gLocalState));
    }
    else
    {
        StatePacket emptyAck = {};
        emptyAck.deviceId = gSelfId;
        radio.writeAckPayload(1, &emptyAck, sizeof(emptyAck));
    }

    return true;
}