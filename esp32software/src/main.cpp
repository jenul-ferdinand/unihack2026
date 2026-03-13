#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#include "rf24_link.h"
#include "imu.h"

#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);
LinkState linkState;

// change this on one board
LinkRole myRole = LINK_SLAVE;
// LinkRole myRole = LINK_SLAVE;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Booting...");

    if (!imu_begin())
    {
        Serial.println("IMU begin failed");
        while (1) delay(100);
    }

    if (!rf24_link_begin(radio, myRole))
    {
        Serial.println("RF24 init failed");
        while (1) delay(100);
    }

    // prepare first packet snapshot
    rf24_link_update_local_packet(linkState);
}

void loop()
{
    imu_update();
    rf24_link_update_local_packet(linkState);

    if (myRole == LINK_MASTER)
    {
        rf24_link_master_step(radio, linkState);
    }
    else
    {
        rf24_link_slave_step(radio, linkState);
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 250)
    {
        lastPrint = millis();

        rf24_link_print_status(myRole == LINK_MASTER ? "MASTER" : "SLAVE", linkState);
        rf24_link_print_packet("LOCAL", linkState.my_packet);

        if (linkState.connected)
        {
            rf24_link_print_packet("PEER ", linkState.peer_packet);
        }

        Serial.println();
    }

    delay(10);
}