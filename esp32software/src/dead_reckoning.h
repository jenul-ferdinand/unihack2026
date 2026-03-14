#pragma once

#include <Arduino.h>
#include "link.h"

struct DeadReckoningInput
{
    float linAccelBodyX = 0.0f;
    float linAccelBodyY = 0.0f;
    float linAccelBodyZ = 0.0f;
    float gyroHeadingDeg = 0.0f;
    float compareRollDeg = 0.0f;
    bool stationary = false;
    bool zupt = false;
};

struct DeadReckoningState
{
    float posX = 0.0f;
    float posY = 0.0f;
    float posZ = 0.0f;
    float velX = 0.0f;
    float velY = 0.0f;
    float velZ = 0.0f;
    float speedMps = 0.0f;
    float accelMss = 0.0f;
    float stepMeters = 0.0f;
    float gyroHeadingDeg = 0.0f;
    float quantizedMoveHeadingDeg = 0.0f;
    float snappedCompareHeadingDeg = 0.0f;
    float snappedCompareRollDeg = 0.0f;
    const char *snappedFacingName = "forward";
    bool peerTruthApplied = false;
    bool hasMovedSinceLastLock = false;
    bool lockedAfterMove = false;
    uint32_t lastUpdateUs = 0;
};

struct DeadReckoningPeerView
{
    float dx = 0.0f;
    float dy = 0.0f;
    float dz = 0.0f;
    float distance = 0.0f;
    float bearingWorldDeg = 0.0f;
    float bearingLocalDeg = 0.0f;
    bool isClose = false;
    uint8_t directionCode = LINK_PEER_UNKNOWN;
    uint8_t directionConfidence = 0;
};

void deadReckoningInit(DeadReckoningState &state, uint32_t nowUs);
void deadReckoningUpdate(DeadReckoningState &state, const DeadReckoningInput &input, uint32_t nowUs);
void deadReckoningGetSharedPosition(const DeadReckoningState &state,
                                    bool sharedFrameLocked,
                                    float sharedYawOffsetDeg,
                                    float &x,
                                    float &y,
                                    float &z);
void deadReckoningComputePeerView(const DeadReckoningState &state,
                                  bool sharedFrameLocked,
                                  float sharedYawOffsetDeg,
                                  float localYawDeg,
                                  float peerX,
                                  float peerY,
                                  float peerZ,
                                  DeadReckoningPeerView &view);
void deadReckoningApplyPeerTruth(DeadReckoningState &state,
                                 bool sharedFrameLocked,
                                 float sharedYawOffsetDeg,
                                 float peerX,
                                 float peerY,
                                 float peerZ,
                                 float peerFacingHeadingDeg,
                                 uint8_t peerDirectionCode,
                                 uint8_t peerConfidence,
                                 bool peerStationary,
                                 bool peerLockedAfterMove,
                                 bool localStationary,
                                 bool localLockedAfterMove,
                                 const DeadReckoningPeerView &localView);
const char *deadReckoningDirectionName(uint8_t directionCode);
