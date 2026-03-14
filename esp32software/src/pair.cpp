#include "pair.h"
#include <string.h>
#include "debug_config.h"

static const byte DISCOVERY_ADDR[6] = "PAIR0";

static const byte NODE1_ADDR[6] = "NODE1";
static const byte NODE2_ADDR[6] = "NODE2";

static const byte *nodeAddr(uint8_t id)
{
  if (id == 1)
    return NODE1_ADDR;
  else
    return NODE2_ADDR;
}
enum MsgType : uint8_t
{
  MSG_DISCOVER = 1,
  MSG_OFFER = 2,
  MSG_CONFIRM = 3,
  MSG_CONFIRMED = 4
};

struct PairMsg
{
  uint8_t type;
  uint8_t fromId;
  uint8_t toId;
  uint8_t reserved;
};

enum PairState : uint8_t
{
  ST_SEARCH = 0,
  ST_WAIT_OFFER,
  ST_WAIT_CONFIRMED,
  ST_PAIRED
};

struct InternalState
{
  PairState state = ST_SEARCH;
  unsigned long lastTxMs = 0;
  unsigned long lastRxMs = 0;
};

static InternalState s;

static const unsigned long DISCOVER_INTERVAL_MS = 250;
static const unsigned long CONFIRM_INTERVAL_MS = 250;
static const unsigned long SEARCH_TIMEOUT_MS = 2000;

static void startDiscoveryListening(RF24 &radio)
{
  radio.flush_tx();
  radio.flush_rx();
  radio.openReadingPipe(1, DISCOVERY_ADDR);
  radio.openWritingPipe(DISCOVERY_ADDR);
  radio.startListening();
}

static bool readPacket(RF24 &radio, PairMsg &msg)
{
  if (!radio.available())
    return false;

  uint8_t len = radio.getDynamicPayloadSize();
  if (len != sizeof(PairMsg))
  {
    radio.flush_rx();
    return false;
  }

  radio.read(&msg, sizeof(msg));

#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
  Serial.print("RX type=");
  Serial.print(msg.type);
  Serial.print(" from=");
  Serial.print(msg.fromId);
  Serial.print(" to=");
  Serial.println(msg.toId);
#endif

  return true;
}

static bool sendPacket(RF24 &radio, const PairMsg &msg, PairMsg *ackOut = nullptr)
{
  radio.stopListening();

  bool ok = radio.write(&msg, sizeof(msg));

#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
  Serial.print("TX type=");
  Serial.print(msg.type);
  Serial.print(" from=");
  Serial.print(msg.fromId);
  Serial.print(" to=");
  Serial.print(msg.toId);
  Serial.println(ok ? " (ok)" : " (fail)");
#endif

  bool gotAckPayload = false;
  if (ok && radio.isAckPayloadAvailable() && ackOut != nullptr)
  {
    uint8_t len = radio.getDynamicPayloadSize();
    if (len == sizeof(PairMsg))
    {
      radio.read(ackOut, sizeof(PairMsg));

#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
      Serial.print("ACK type=");
      Serial.print(ackOut->type);
      Serial.print(" from=");
      Serial.print(ackOut->fromId);
      Serial.print(" to=");
      Serial.println(ackOut->toId);
#endif

      gotAckPayload = true;
    }
    else
    {
      radio.flush_rx();
    }
  }

  radio.startListening();
  return gotAckPayload;
}

void pairingBegin(RF24 &radio, PairContext &ctx, uint8_t selfId)
{
  ctx.selfId = selfId;
  ctx.peerId = 0;
  ctx.paired = false;

  s.state = ST_SEARCH;
  s.lastTxMs = 0;
  s.lastRxMs = millis();

  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setRetries(5, 15);

  startDiscoveryListening(radio);

#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
  Serial.print("Pairing begin, selfId=");
  Serial.println(selfId);
#endif
}

static void responderHandle(RF24 &radio, PairContext &ctx, const PairMsg &rx)
{
  if (ctx.selfId != 2)
    return;

  if (rx.type == MSG_DISCOVER && rx.toId == 0)
  {
    ctx.peerId = rx.fromId;

    PairMsg ack{};
    ack.type = MSG_OFFER;
    ack.fromId = ctx.selfId;
    ack.toId = ctx.peerId;

    radio.writeAckPayload(1, &ack, sizeof(ack));
    s.lastRxMs = millis();

#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
    Serial.println("Queued OFFER");
#endif

    // IMPORTANT:
    // After offering, switch to our dedicated receive address
    // so we can hear the master's CONFIRM packet.
    radio.stopListening();
    radio.openReadingPipe(1, nodeAddr(ctx.selfId));
    radio.startListening();

#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
    Serial.print("Responder listening on dedicated addr for selfId=");
    Serial.println(ctx.selfId);
#endif
  }
  else if (rx.type == MSG_CONFIRM && rx.toId == ctx.selfId)
  {
    ctx.peerId = rx.fromId;

    PairMsg ack{};
    ack.type = MSG_CONFIRMED;
    ack.fromId = ctx.selfId;
    ack.toId = ctx.peerId;

    radio.writeAckPayload(1, &ack, sizeof(ack));
    s.lastRxMs = millis();

    if (!ctx.paired)
    {
      ctx.paired = true;
      s.state = ST_PAIRED;
#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
      Serial.println("Pairing complete (responder)");
#endif
    }
    else
    {
#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
      Serial.println("Re-queued CONFIRMED");
#endif
    }
  }
}

void pairingUpdate(RF24 &radio, PairContext &ctx)
{
  PairMsg rx{};

  while (readPacket(radio, rx))
  {
    responderHandle(radio, ctx, rx);
  }

  if (ctx.selfId == 2)
  {
    return;
  }

  unsigned long now = millis();

  if (s.state == ST_SEARCH)
  {
    if (now - s.lastTxMs >= DISCOVER_INTERVAL_MS)
    {
      PairMsg tx{};
      tx.type = MSG_DISCOVER;
      tx.fromId = ctx.selfId;
      tx.toId = 0;

      PairMsg ack{};
      bool gotAck = sendPacket(radio, tx, &ack);
      s.lastTxMs = now;

      if (gotAck && ack.type == MSG_OFFER && ack.toId == ctx.selfId)
      {
        ctx.peerId = ack.fromId;
        s.state = ST_WAIT_CONFIRMED;
#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
        Serial.print("Found peer ");
        Serial.println(ctx.peerId);
#endif
      }
    }
  }
  else if (s.state == ST_WAIT_CONFIRMED)
  {
    if (now - s.lastTxMs >= CONFIRM_INTERVAL_MS)
    {
      PairMsg tx{};
      tx.type = MSG_CONFIRM;
      tx.fromId = ctx.selfId;
      tx.toId = ctx.peerId;

      PairMsg ack{};
      bool gotAck = sendPacket(radio, tx, &ack);
      s.lastTxMs = now;

      if (gotAck && ack.type == MSG_CONFIRMED && ack.toId == ctx.selfId)
      {
        ctx.paired = true;
        s.state = ST_PAIRED;
#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
        Serial.println("Pairing complete (initiator)");
#endif
      }
    }

    if (now - s.lastTxMs > SEARCH_TIMEOUT_MS)
    {
      s.state = ST_SEARCH;
      ctx.peerId = 0;
#if DEBUG_SERIAL_ENABLE && DEBUG_PAIRING
      Serial.println("Timeout, back to search");
#endif
    }
  }
}

bool pairingIsComplete(const PairContext &ctx)
{
  return ctx.paired;
}
