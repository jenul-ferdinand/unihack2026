#pragma once
#include <Arduino.h>
#include <RF24.h>

enum NodeRole
{
    ROLE_A,
    ROLE_B
};

bool runPairing(RF24 &radio, NodeRole role);