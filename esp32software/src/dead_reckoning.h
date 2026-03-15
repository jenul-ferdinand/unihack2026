#pragma once

#include <Arduino.h>
#include "link.h"

// A coarse motion estimate built for peer-relative direction, not accurate
// navigation. The update step quantizes heading to cardinals and uses a heavily
// damped scalar speed so the result remains stable enough for two devices to
// agree on "peer is left/right/front/back".

struct DeadReckoningInput
{
    // Body-frame linear acceleration from the IMU layer.
    float linAccelBodyX = 0.0f;
    float linAccelBodyY = 0.0f;
    float linAccelBodyZ = 0.0f;

    // Heading used for movement integration and pitch used for posture/facing
    // classification.
    float gyroHeadingDeg = 0.0f;
    float comparePitchDeg = 0.0f;

    // Stationary flags from the sketch-level motion detector.
    bool stationary = false;
    bool zupt = false;
};

struct DeadReckoningState
{
    // Local dead-reckoned pose in the device's own frame.
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
    float snappedComparePitchDeg = 0.0f;
    float lastAcceptedComparePitchDeg = 0.0f;
    // Facing bucket inferred from comparePitchDeg.
    const char *snappedFacingName = "forward";
    bool hasAcceptedComparePitch = false;

    // If true, the local estimate was corrected using peer-reported truth.
    bool peerTruthApplied = false;
    bool hasMovedSinceLastLock = false;
    bool lockedAfterMove = false;
    uint32_t lastUpdateUs = 0;
};

struct DeadReckoningPeerView
{
    // Relative position of the peer from the local device's perspective.
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
void deadReckoningAnchorPosition(DeadReckoningState &state, float x, float y, float z);

// Convert local dead-reckoned position into the shared frame once both devices
// have agreed on the yaw offset between their local frames.
void deadReckoningGetSharedPosition(const DeadReckoningState &state,
                                    bool sharedFrameLocked,
                                    float sharedYawOffsetDeg,
                                    float &x,
                                    float &y,
                                    float &z);

// Compute the peer's relative direction from local pose + the latest peer pose.
void deadReckoningComputePeerView(const DeadReckoningState &state,
                                  bool sharedFrameLocked,
                                  float sharedYawOffsetDeg,
                                  float localYawDeg,
                                  float peerX,
                                  float peerY,
                                  float peerZ,
                                  DeadReckoningPeerView &view);

// If the peer has a better directional observation, snap our local estimate so
// both devices converge on a more consistent relative layout.
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
                                 const DeadReckoningPeerView &localView);
const char *deadReckoningDirectionName(uint8_t directionCode);
