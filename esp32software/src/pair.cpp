#include "pair.h"
#include <string.h>

static const byte ADDR_A[6] = "NODE1";
static const byte ADDR_B[6] = "NODE2";

struct PairPacket
{
    char text[16];
};

static void configureRadio(RF24 &radio)
{
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.setRetries(5, 15);
    radio.flush_tx();
    radio.flush_rx();
}

bool runPairing(RF24 &radio, NodeRole role)
{
    configureRadio(radio);

    if (role == ROLE_A)
    {
        radio.openReadingPipe(1, ADDR_A);
        radio.openWritingPipe(ADDR_B);
        radio.stopListening();

        Serial.println("Starting pairing as ROLE_A");

        PairPacket tx = {"PING"};
        PairPacket ack = {};

        while (true)
        {
            bool ok = radio.write(&tx, sizeof(tx));

            Serial.print("TX: ");
            Serial.print(tx.text);
            Serial.println(ok ? " (ok)" : " (fail)");

            if (ok && radio.isAckPayloadAvailable())
            {
                uint8_t len = radio.getDynamicPayloadSize();
                if (len == sizeof(ack))
                {
                    radio.read(&ack, sizeof(ack));

                    Serial.print("ACK: ");
                    Serial.println(ack.text);

                    if (strcmp(ack.text, "PONG") == 0)
                    {
                        Serial.println("Pairing complete (ROLE_A)");
                        return true;
                    }
                }
                else
                {
                    radio.flush_rx();
                }
            }

            delay(250);
        }
    }
    else
    {
        radio.openReadingPipe(1, ADDR_B);
        radio.openWritingPipe(ADDR_A);
        radio.startListening();

        Serial.println("Starting pairing as ROLE_B");

        PairPacket rx = {};
        PairPacket ack = {"PONG"};

        while (true)
        {
            if (radio.available())
            {
                uint8_t len = radio.getDynamicPayloadSize();
                if (len == sizeof(rx))
                {
                    radio.read(&rx, sizeof(rx));

                    Serial.print("RX: ");
                    Serial.println(rx.text);

                    if (strcmp(rx.text, "PING") == 0)
                    {
                        radio.writeAckPayload(1, &ack, sizeof(ack));
                        Serial.println("Queued ACK: PONG");
                        Serial.println("Pairing complete (ROLE_B)");
                        return true;
                    }
                }
                else
                {
                    radio.flush_rx();
                }
            }

            delay(5);
        }
    }
}