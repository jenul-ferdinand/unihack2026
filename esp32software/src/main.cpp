#include <Arduino.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <RF24.h>
#include <math.h>
#include <memory>

#include "imu.h"
#include "link.h"
#include "time_sync.h"
#include "imu_accel.h"
#include "dead_reckoning.h"
#include "debug_config.h"

#define CE_PIN 4
#define CSN_PIN 5

// CHANGE THESE PER DEVICE
#define SELF_ID 2
#define IS_TIME_MASTER 0

#if IS_TIME_MASTER
#define IS_POLLER 1
#else
#define IS_POLLER 0
#endif

RF24 radio(CE_PIN, CSN_PIN);

static uint16_t gSeq = 0;
static StatePacket localPkt = {};
static StatePacket peerPkt = {};
static TimeSyncState gTimeSync;
static bool gHavePeerPacket = false;
static bool gPeerPacketStale = false;
static unsigned long gLastPeerPacketMs = 0;

// ---------- Tunable thresholds ----------
static const float G_MS2 = 9.80665f;
static const float STATIONARY_GYRO_DPS = 2.0f;
static const float STATIONARY_ACC_ERR_MS2 = 0.35f;
static const float ZUPT_GYRO_DPS = 1.2f;
static const float ZUPT_ACC_ERR_MS2 = 0.20f;

static const int STILL_SAMPLES_TO_LATCH = 8;
static const int MOVE_SAMPLES_TO_RELEASE = 4;

// Hard reset after being still a bit
static const unsigned long STILL_RESET_MS = 1200;

// Shared-frame alignment
static bool gInitialYawLocked = false;
static float gInitialYawDeg = 0.0f;
static bool gSharedFrameLocked = false;
static float gSharedYawOffsetDeg = 0.0f;

// ---------- Local estimator/debug state ----------
struct MotionState
{
    // Raw sensor/debug
    float rawAx = 0, rawAy = 0, rawAz = 0;
    float rawGx = 0, rawGy = 0, rawGz = 0;
    float rawMx = 0, rawMy = 0, rawMz = 0;

    float linAxB = 0, linAyB = 0, linAzB = 0;
    float linAxW = 0, linAyW = 0, linAzW = 0;

    // Raw pose from IMU layer, shifted so startup = origin
    float rawPosX = 0, rawPosY = 0, rawPosZ = 0;
    float rawVelX = 0, rawVelY = 0, rawVelZ = 0;

    // Clamped pose
    float clampPosX = 0, clampPosY = 0, clampPosZ = 0;
    float clampVelX = 0, clampVelY = 0, clampVelZ = 0;

    // Persistent correction offset:
    // corrected = raw + correction
    float corrX = 0, corrY = 0, corrZ = 0;

    // Startup origin from IMU layer
    float originX = 0, originY = 0, originZ = 0;
    bool originSet = false;

    bool stationary = false;
    bool zupt = false;

    float gyroNorm = 0;
    float accNorm = 0;
    float accErr = 0;

    int stillCount = 0;
    int moveCount = 0;

    bool imuHoldActive = false;
    unsigned long stillSinceMs = 0;
    bool resetDoneThisStillness = false;
    bool prevHold;

    float rawRoll = 0, rawPitch = 0, rawYaw = 0;
    float clampRoll = 0, clampPitch = 0, clampYaw = 0;
    float corrRoll = 0, corrPitch = 0, corrYaw = 0;

    float orientOffRoll = 0, orientOffPitch = 0, orientOffYaw = 0;

    float holdRollRef = 0, holdPitchRef = 0, holdYawRef = 0;
    bool angleHoldRefValid;

    bool holdLatched = false;
};

static MotionState gMotion;

static DeadReckoningState gDeadReckoning;
static DeadReckoningPeerView gDeadPeerView;

// ---------- Timing ----------
static unsigned long gLastDebugMs = 0;
static unsigned long gLastTelemetryLogMs = 0;
static unsigned long gLastWifiCheckMs = 0;
static constexpr unsigned long PEER_PACKET_STALE_MS = 500;

// ---------- Telemetry persistence / upload ----------
static constexpr const char *WIFI_SSID = "Christopher's A35";
static constexpr const char *WIFI_PASSWORD = "its a secret";
static constexpr const char *WEBHOOK_URL = "http://10.43.196.51:8080/api";
static constexpr const char *TELEMETRY_LOG_PATH = "/telemetry.bin";
static constexpr const char *TELEMETRY_TMP_PATH = "/telemetry.tmp";
static constexpr unsigned long TELEMETRY_LOG_INTERVAL_MS = 100;
static constexpr unsigned long WIFI_CHECK_INTERVAL_MS = 10000;
static constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 5000;
static constexpr size_t TELEMETRY_MAX_FILE_BYTES = 7 * 1024 * 1024;
static constexpr size_t TELEMETRY_TRIM_TARGET_BYTES = 5 * 1024 * 1024;
static constexpr unsigned long TELEMETRY_SEND_READY_DELAY_MS = 2000;
static constexpr size_t TELEMETRY_POST_BATCH_RECORDS = 24;

struct __attribute__((packed)) TelemetryVec3
{
    float x;
    float y;
    float z;
};

struct __attribute__((packed)) TelemetryLogRecord
{
    uint32_t loggedAtMs;

    uint8_t initialYawLocked;
    float initialYawDeg;
    uint8_t sharedFrameLocked;
    float sharedYawOffsetDeg;

    TelemetryVec3 rawPos;
    TelemetryVec3 rawVel;
    TelemetryVec3 corrOffset;
    TelemetryVec3 clampedPos;
    TelemetryVec3 clampedVel;

    uint8_t peerId;
    uint32_t peerTimeUs;
    TelemetryVec3 peerPos;
    float peerYawDeg;
    float peerInitYawDeg;
    float peerSpeedMps;

    TelemetryVec3 deltaXyz;
    float distanceM;
    float bearingWorldDeg;
    float bearingLocalDeg;
};

static_assert(sizeof(TelemetryLogRecord) == 127, "TelemetryLogRecord layout changed unexpectedly");

static bool gTelemetryPausedForSend = false;
static bool gTelemetryReadyPrinted = false;
static unsigned long gTelemetryReadyAtMs = 0;

// ---------- Utility ----------
static float wrapAngleDeg(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

static float vecNorm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float degToRad(float deg)
{
    return deg * PI / 180.0f;
}

static bool wifiConfigured()
{
    return WIFI_SSID[0] != '\0' && WEBHOOK_URL[0] != '\0';
}

static void appendFloat(String &json, float value, int decimals)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    json += buffer;
}

static void appendVec3Json(String &json, const char *key, const TelemetryVec3 &vec, int decimals)
{
    json += "\"";
    json += key;
    json += "\":{";
    json += "\"x\":";
    appendFloat(json, vec.x, decimals);
    json += ",\"y\":";
    appendFloat(json, vec.y, decimals);
    json += ",\"z\":";
    appendFloat(json, vec.z, decimals);
    json += "}";
}

static TelemetryVec3 makeVec3(float x, float y, float z)
{
    TelemetryVec3 vec{};
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
}

static TelemetryLogRecord captureTelemetryRecord()
{
    TelemetryLogRecord record{};
    record.loggedAtMs = millis();

    record.initialYawLocked = gInitialYawLocked ? 1 : 0;
    record.initialYawDeg = gInitialYawDeg;
    record.sharedFrameLocked = gSharedFrameLocked ? 1 : 0;
    record.sharedYawOffsetDeg = gSharedYawOffsetDeg;

    record.rawPos = makeVec3(gMotion.rawPosX, gMotion.rawPosY, gMotion.rawPosZ);
    record.rawVel = makeVec3(gMotion.rawVelX, gMotion.rawVelY, gMotion.rawVelZ);
    record.corrOffset = makeVec3(gMotion.corrX, gMotion.corrY, gMotion.corrZ);
    record.clampedPos = makeVec3(gMotion.clampPosX, gMotion.clampPosY, gMotion.clampPosZ);
    record.clampedVel = makeVec3(gMotion.clampVelX, gMotion.clampVelY, gMotion.clampVelZ);

    record.peerId = peerPkt.deviceId;
    record.peerTimeUs = peerPkt.timeUs;
    record.peerPos = makeVec3(peerPkt.posX, peerPkt.posY, peerPkt.posZ);
    record.peerYawDeg = peerPkt.yawDeg;
    record.peerInitYawDeg = peerPkt.initYawDeg;
    record.peerSpeedMps = peerPkt.speedMps;

    record.deltaXyz = makeVec3(gDeadPeerView.dx, gDeadPeerView.dy, gDeadPeerView.dz);
    record.distanceM = gDeadPeerView.distance;
    record.bearingWorldDeg = gDeadPeerView.bearingWorldDeg;
    record.bearingLocalDeg = gDeadPeerView.bearingLocalDeg;

    return record;
}

static String telemetryRecordToJson(const TelemetryLogRecord &record)
{
    String json;
    json.reserve(700);

    json += "{";
    json += "\"shared_frame\":{";
    json += "\"initial_yaw_locked\":";
    json += String(record.initialYawLocked);
    json += ",\"initial_yaw_deg\":";
    appendFloat(json, record.initialYawDeg, 2);
    json += ",\"shared_frame_locked\":";
    json += String(record.sharedFrameLocked);
    json += ",\"shared_yaw_offset_deg\":";
    appendFloat(json, record.sharedYawOffsetDeg, 2);
    json += "},";

    json += "\"position_velocity\":{";
    appendVec3Json(json, "raw_pos", record.rawPos, 3);
    json += ",";
    appendVec3Json(json, "raw_vel", record.rawVel, 3);
    json += ",";
    appendVec3Json(json, "corr_offset", record.corrOffset, 3);
    json += ",";
    appendVec3Json(json, "clamped_pos", record.clampedPos, 3);
    json += ",";
    appendVec3Json(json, "clamped_vel", record.clampedVel, 3);
    json += "},";

    json += "\"peer_state\":{";
    json += "\"peer_id\":";
    json += String(record.peerId);
    json += ",\"peer_t_us\":";
    json += String(record.peerTimeUs);
    json += ",";
    appendVec3Json(json, "peer_pos", record.peerPos, 3);
    json += ",\"peer_yaw_deg\":";
    appendFloat(json, record.peerYawDeg, 2);
    json += ",\"peer_init_yaw_deg\":";
    appendFloat(json, record.peerInitYawDeg, 2);
    json += ",\"peer_speed_mps\":";
    appendFloat(json, record.peerSpeedMps, 3);
    json += "},";

    json += "\"relative_to_peer\":{";
    appendVec3Json(json, "delta_xyz", record.deltaXyz, 3);
    json += ",\"distance_m\":";
    appendFloat(json, record.distanceM, 3);
    json += ",\"bearing_world_deg\":";
    appendFloat(json, record.bearingWorldDeg, 2);
    json += ",\"bearing_local_deg\":";
    appendFloat(json, record.bearingLocalDeg, 2);
    json += "}";
    json += "}";

    return json;
}

static void trimTelemetryLogIfNeeded()
{
    File src = SPIFFS.open(TELEMETRY_LOG_PATH, FILE_READ);
    if (!src)
        return;

    const size_t currentSize = src.size();
    if (currentSize <= TELEMETRY_MAX_FILE_BYTES)
    {
        src.close();
        return;
    }

    const size_t bytesToKeep = min(currentSize, TELEMETRY_TRIM_TARGET_BYTES);
    const size_t startOffset = currentSize - bytesToKeep;
    src.seek(startOffset, SeekSet);

    File dst = SPIFFS.open(TELEMETRY_TMP_PATH, FILE_WRITE);
    if (!dst)
    {
        src.close();
        return;
    }

    uint8_t buffer[512];
    while (src.available())
    {
        const size_t chunk = src.read(buffer, sizeof(buffer));
        if (chunk == 0)
            break;
        dst.write(buffer, chunk);
    }

    src.close();
    dst.close();

    SPIFFS.remove(TELEMETRY_LOG_PATH);
    SPIFFS.rename(TELEMETRY_TMP_PATH, TELEMETRY_LOG_PATH);
}

static void appendTelemetryLogRecord(const TelemetryLogRecord &record)
{
    File file = SPIFFS.open(TELEMETRY_LOG_PATH, FILE_APPEND);
    if (!file)
        return;

    file.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record));
    file.close();
    trimTelemetryLogIfNeeded();
}

static void discardTelemetryPrefix(size_t bytesToDrop)
{
    File src = SPIFFS.open(TELEMETRY_LOG_PATH, FILE_READ);
    if (!src)
        return;

    const size_t totalSize = src.size();
    if (bytesToDrop >= totalSize)
    {
        src.close();
        SPIFFS.remove(TELEMETRY_LOG_PATH);
        return;
    }

    src.seek(bytesToDrop, SeekSet);

    File dst = SPIFFS.open(TELEMETRY_TMP_PATH, FILE_WRITE);
    if (!dst)
    {
        src.close();
        return;
    }

    uint8_t buffer[512];
    while (src.available())
    {
        const size_t chunk = src.read(buffer, sizeof(buffer));
        if (chunk == 0)
            break;
        dst.write(buffer, chunk);
    }

    src.close();
    dst.close();

    SPIFFS.remove(TELEMETRY_LOG_PATH);
    SPIFFS.rename(TELEMETRY_TMP_PATH, TELEMETRY_LOG_PATH);
}

static bool ensureWifiConnected()
{
    if (!wifiConfigured())
    {
        Serial.println("WiFi check skipped: SSID or webhook URL not configured.");
        return false;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("WiFi already connected. IP=");
        Serial.println(WiFi.localIP());
        return true;
    }

    Serial.print("Attempting WiFi connection to SSID: ");
    Serial.println(WIFI_SSID);
    Serial.print("WiFi status before begin: ");
    Serial.println(WiFi.status());
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS)
    {
        Serial.print("WiFi connecting, status=");
        Serial.println(WiFi.status());
        delay(100);
    }

    Serial.print("WiFi connect result status=");
    Serial.println(WiFi.status());
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("WiFi connected. IP=");
        Serial.println(WiFi.localIP());
    }

    return WiFi.status() == WL_CONNECTED;
}

static bool hasQueuedTelemetry()
{
    File file = SPIFFS.open(TELEMETRY_LOG_PATH, FILE_READ);
    if (!file)
        return false;

    const bool hasData = file.size() >= sizeof(TelemetryLogRecord);
    file.close();
    return hasData;
}

static void printQueuedTelemetryStats()
{
    File file = SPIFFS.open(TELEMETRY_LOG_PATH, FILE_READ);
    if (!file)
    {
        Serial.println("Queued telemetry: file missing or empty.");
        return;
    }

    const size_t bytes = file.size();
    const size_t samples = bytes / sizeof(TelemetryLogRecord);
    file.close();

    Serial.print("Queued telemetry bytes=");
    Serial.print(bytes);
    Serial.print(", samples=");
    Serial.println(samples);
}

static bool postTelemetryBatch(size_t &sentRecordsOut)
{
    sentRecordsOut = 0;

    if (!wifiConfigured() || !ensureWifiConnected())
        return false;

    File file = SPIFFS.open(TELEMETRY_LOG_PATH, FILE_READ);
    if (!file)
        return false;

    const size_t fileSize = file.size();
    const size_t recordSize = sizeof(TelemetryLogRecord);
    const size_t availableRecords = fileSize / recordSize;
    if (availableRecords == 0)
    {
        file.close();
        return true;
    }

    const size_t batchRecords = min(availableRecords, TELEMETRY_POST_BATCH_RECORDS);
    String payload;
    payload.reserve(128 + (batchRecords * 700));
    payload += "{";
    payload += "\"device_id\":";
    payload += String(SELF_ID);
    payload += ",\"sample_count\":";
    payload += String(batchRecords);
    payload += ",\"samples\":[";

    TelemetryLogRecord record{};
    size_t sentRecords = 0;
    while (sentRecords < batchRecords &&
           file.read(reinterpret_cast<uint8_t *>(&record), sizeof(record)) == sizeof(record))
    {
        if (sentRecords > 0)
            payload += ",";
        payload += telemetryRecordToJson(record);
        sentRecords++;
    }

    file.close();
    payload += "]}";

    if (sentRecords == 0)
        return true;

    HTTPClient http;
    Serial.print("Posting telemetry to ");
    Serial.println(WEBHOOK_URL);
    Serial.print("Telemetry batch samples=");
    Serial.println(sentRecords);
    Serial.print("Telemetry payload bytes=");
    Serial.println(payload.length());

    http.begin(WEBHOOK_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Accept", "application/json");
    http.useHTTP10(true);

    std::unique_ptr<char[]> requestBody(new char[payload.length() + 1]);
    memcpy(requestBody.get(), payload.c_str(), payload.length() + 1);

    const int responseCode = http.POST(reinterpret_cast<uint8_t *>(requestBody.get()), payload.length());
    const String responseBody = http.getString();
    http.end();

    Serial.print("Telemetry POST response code=");
    Serial.println(responseCode);
    Serial.print("Telemetry POST response body=");
    Serial.println(responseBody);

    if (responseCode < 200 || responseCode >= 300)
        return false;

    discardTelemetryPrefix(sentRecords * sizeof(TelemetryLogRecord));
    sentRecordsOut = sentRecords;
    return true;
}

static void maybeLogTelemetry()
{
    if (gTelemetryPausedForSend)
        return;

    const unsigned long nowMs = millis();
    if (nowMs - gLastTelemetryLogMs < TELEMETRY_LOG_INTERVAL_MS)
        return;

    gLastTelemetryLogMs = nowMs;
    const TelemetryLogRecord record = captureTelemetryRecord();
    appendTelemetryLogRecord(record);
}

static void maybeUploadTelemetry()
{
    const unsigned long nowMs = millis();

    if (gTelemetryPausedForSend)
    {
        if (!gTelemetryReadyPrinted)
        {
            Serial.println("WiFi connected. Telemetry paused. Ready to send queued JSON in 2 seconds.");
            gTelemetryReadyPrinted = true;
            gTelemetryReadyAtMs = nowMs;
        }

        if (nowMs - gTelemetryReadyAtMs < TELEMETRY_SEND_READY_DELAY_MS)
            return;

        size_t sentRecords = 0;
        const bool sent = postTelemetryBatch(sentRecords);
        if (sent)
        {
            Serial.print("Queued telemetry batch sent. Samples sent=");
            Serial.println(sentRecords);
        }
        else
        {
            Serial.println("Queued telemetry send failed.");
        }

        if (!hasQueuedTelemetry() || !sent)
        {
            gTelemetryPausedForSend = false;
            gTelemetryReadyPrinted = false;
            gTelemetryReadyAtMs = 0;
            gLastWifiCheckMs = nowMs;
        }
        else
        {
            gTelemetryReadyPrinted = true;
            gTelemetryReadyAtMs = nowMs;
            Serial.println("More queued telemetry remains. Sending next batch in 2 seconds.");
        }
        return;
    }

    if (nowMs - gLastWifiCheckMs < WIFI_CHECK_INTERVAL_MS)
        return;

    gLastWifiCheckMs = nowMs;
    Serial.println("Running 10 second WiFi/telemetry check.");
    printQueuedTelemetryStats();

    if (!hasQueuedTelemetry())
    {
        Serial.println("No queued telemetry to send.");
        return;
    }

    if (!ensureWifiConnected())
    {
        Serial.println("WiFi not connected, telemetry send deferred.");
        return;
    }

    gTelemetryPausedForSend = true;
    gTelemetryReadyPrinted = false;
    gTelemetryReadyAtMs = nowMs;
}

static void rotate2D(float x, float y, float yawDeg, float &outX, float &outY)
{
    const float r = degToRad(yawDeg);
    const float c = cosf(r);
    const float s = sinf(r);
    outX = c * x - s * y;
    outY = s * x + c * y;
}

static uint8_t buildFlags()
{
    uint8_t flags = 0;

    // bit0 = stance / stationary-ish
    // bit1 = ZUPT active
    // bit2 = stationary
    // bit3 = heading / time reliable
    // bit4 = moved, then stationary lock is trusted
    // bit5 = reserved
    // bit6 = initial yaw valid / shared frame info valid
    // bit7 = shared frame locked locally

    if (gMotion.stationary)
        flags |= (1 << 0);
    if (gMotion.zupt)
        flags |= (1 << 1);
    if (gMotion.stationary)
        flags |= (1 << 2);

#if IS_TIME_MASTER
    flags |= (1 << 3);
#else
    if (timeSyncLocked(gTimeSync))
        flags |= (1 << 3);
#endif

    if (gInitialYawLocked)
        flags |= (1 << 6);
    if (gSharedFrameLocked)
        flags |= (1 << 7);
    if (gDeadReckoning.lockedAfterMove)
        flags |= (1 << 4);

    return flags;
}

static uint32_t sharedNowUs()
{
    const uint32_t nowLocal = micros();

#if IS_TIME_MASTER
    return nowLocal;
#else
    return timeSyncNowUs(gTimeSync, nowLocal);
#endif
}

static void captureImuState()
{
    imu_getRawAccel(gMotion.rawAx, gMotion.rawAy, gMotion.rawAz);
    imu_getRawGyro(gMotion.rawGx, gMotion.rawGy, gMotion.rawGz);
    imu_getRawMag(gMotion.rawMx, gMotion.rawMy, gMotion.rawMz);

    imu_getLinearAccel(gMotion.linAxB, gMotion.linAyB, gMotion.linAzB);
    imu_getLinearAccelWorld(gMotion.linAxW, gMotion.linAyW, gMotion.linAzW);
    imu_getEuler(gMotion.rawRoll, gMotion.rawPitch, gMotion.rawYaw);

    float px, py, pz;
    float vx, vy, vz;

    imu_getPosition(px, py, pz);
    imu_getVelocity(vx, vy, vz);

    if (!gMotion.originSet)
    {
        gMotion.originSet = true;
        gMotion.originX = px;
        gMotion.originY = py;
        gMotion.originZ = pz;
    }

    gMotion.rawPosX = px - gMotion.originX;
    gMotion.rawPosY = py - gMotion.originY;
    gMotion.rawPosZ = pz - gMotion.originZ;

    gMotion.rawVelX = vx;
    gMotion.rawVelY = vy;
    gMotion.rawVelZ = vz;
}

static void detectMotionFlags()
{
    gMotion.gyroNorm = vecNorm3(gMotion.rawGx, gMotion.rawGy, gMotion.rawGz);
    gMotion.accNorm = vecNorm3(gMotion.rawAx, gMotion.rawAy, gMotion.rawAz);
    gMotion.accErr = fabsf(gMotion.accNorm - G_MS2);

    const bool stillCandidate =
        (gMotion.gyroNorm < STATIONARY_GYRO_DPS) &&
        (gMotion.accErr < STATIONARY_ACC_ERR_MS2);

    if (stillCandidate)
    {
        gMotion.stillCount++;
        gMotion.moveCount = 0;
    }
    else
    {
        gMotion.moveCount++;
        gMotion.stillCount = 0;
    }

    if (gMotion.stillCount >= STILL_SAMPLES_TO_LATCH)
        gMotion.stationary = true;
    else if (gMotion.moveCount >= MOVE_SAMPLES_TO_RELEASE)
        gMotion.stationary = false;

    gMotion.zupt =
        (gMotion.gyroNorm < ZUPT_GYRO_DPS) &&
        (gMotion.accErr < ZUPT_ACC_ERR_MS2);
}

static void maybeLockInitialYaw()
{
    if (gInitialYawLocked)
        return;

    if (!(gMotion.stationary || gMotion.zupt))
        return;

    if (gMotion.stillSinceMs == 0)
        return;

    if (millis() - gMotion.stillSinceMs < STILL_RESET_MS)
        return;

    gInitialYawDeg = gDeadReckoning.quantizedMoveHeadingDeg;
    gInitialYawLocked = true;

#if IS_TIME_MASTER
    gSharedYawOffsetDeg = 0.0f;
    gSharedFrameLocked = true;
#endif
}

static void maybeLockSharedFrameFromPeer()
{
#if !IS_TIME_MASTER
    if (gSharedFrameLocked)
        return;

    if (!gInitialYawLocked)
        return;

    if ((peerPkt.flags & (1 << 6)) == 0)
        return;

    gSharedYawOffsetDeg = wrapAngleDeg(peerPkt.initYawDeg - gInitialYawDeg);
    gSharedFrameLocked = true;
#endif
}
static void updateStationaryHold()
{
    const uint32_t now = millis();

    // Raw request from detectors
    const bool holdRequest = (gMotion.stationary || gMotion.zupt);
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Hold requested: %d\n", holdRequest);
#endif
    // Movement is the absence of a hold request from the stationary/ZUPT detectors.
    const bool moved = !holdRequest;
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Moved: %d\n", moved);
#endif

    if (holdRequest)
        gMotion.holdLatched = true;
    else if (moved)
        gMotion.holdLatched = false;

#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Hold latched: %d", gMotion.holdLatched);
#endif

    const bool hold = gMotion.holdLatched;
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Hold: %d\n", hold);
#endif

    imu_setStationary(hold);
    gMotion.imuHoldActive = hold;

    const bool enteredHold = (hold && !gMotion.prevHold);
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf("Entered hold: %d\n", enteredHold);
#endif

    const bool exitedHold = (!hold && gMotion.prevHold);

    // Current corrected orientation from raw + correction
    const float correctedRoll = wrapAngleDeg(gMotion.rawRoll + gMotion.corrRoll);
    const float correctedPitch = wrapAngleDeg(gMotion.rawPitch + gMotion.corrPitch);
    const float correctedYaw = wrapAngleDeg(gMotion.rawYaw + gMotion.corrYaw);

    // Debug output
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
    Serial.printf(
        "PITCH DEBUG | raw: %.2f  corr: %.2f  corrected: %.2f\n",
        gMotion.rawPitch,
        gMotion.corrPitch,
        correctedPitch);

    Serial.printf(
        "ROLL DEBUG | raw: %.2f  corr: %.2f  corrected: %.2f\n",
        gMotion.rawRoll,
        gMotion.corrRoll,
        correctedRoll);

    Serial.printf(
        "YAW DEBUG | raw: %.2f  corr: %.2f  corrected: %.2f\n",
        gMotion.rawYaw,
        gMotion.corrYaw,
        correctedYaw);

    Serial.printf(
        "PRE-APPLY | holdRef=(%.2f, %.2f, %.2f) clamp=(%.2f, %.2f, %.2f)\n",
        gMotion.holdRollRef,
        gMotion.holdPitchRef,
        gMotion.holdYawRef,
        gMotion.clampRoll,
        gMotion.clampPitch,
        gMotion.clampYaw);
#endif

    if (enteredHold)
    {
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
        Serial.println("========= RECALCULATING HOLD =========");

        Serial.printf(
            "NEW HOLD REF | roll: %.2f  pitch: %.2f  yaw: %.2f\n",
            correctedRoll,
            correctedPitch,
            correctedYaw);
#endif

        gMotion.holdRollRef = correctedRoll;
        gMotion.holdPitchRef = correctedPitch;
        gMotion.holdYawRef = correctedYaw;

        gMotion.angleHoldRefValid = true;
        gMotion.stillSinceMs = now;
        gMotion.resetDoneThisStillness = false;

        gMotion.clampVelX = 0.0f;
        gMotion.clampVelY = 0.0f;
        gMotion.clampVelZ = 0.0f;

        // Freeze current corrected orientation immediately
        gMotion.clampRoll = correctedRoll;
        gMotion.clampPitch = correctedPitch;
        gMotion.clampYaw = correctedYaw;
    }

    if (hold)
    {
        // Keep clamp velocity zero while held
        gMotion.clampVelX = 0.0f;
        gMotion.clampVelY = 0.0f;
        gMotion.clampVelZ = 0.0f;

        if (!gMotion.resetDoneThisStillness &&
            (now - gMotion.stillSinceMs >= STILL_RESET_MS))
        {
            // Lock clamp position where it currently is
            gMotion.clampPosX = gMotion.rawPosX + gMotion.corrX;
            gMotion.clampPosY = gMotion.rawPosY + gMotion.corrY;
            gMotion.clampPosZ = gMotion.rawPosZ + gMotion.corrZ;

            // Recompute correction so corrected raw position matches held clamp
            gMotion.corrX = gMotion.clampPosX - gMotion.rawPosX;
            gMotion.corrY = gMotion.clampPosY - gMotion.rawPosY;
            gMotion.corrZ = gMotion.clampPosZ - gMotion.rawPosZ;

            gMotion.clampVelX = 0.0f;
            gMotion.clampVelY = 0.0f;
            gMotion.clampVelZ = 0.0f;

            // Preserve the orientation captured when hold was first entered.
            gMotion.clampRoll = gMotion.holdRollRef;
            gMotion.clampPitch = gMotion.holdPitchRef;
            gMotion.clampYaw = gMotion.holdYawRef;

            gMotion.corrRoll = wrapAngleDeg(gMotion.clampRoll - gMotion.rawRoll);
            gMotion.corrPitch = wrapAngleDeg(gMotion.clampPitch - gMotion.rawPitch);
            gMotion.corrYaw = wrapAngleDeg(gMotion.clampYaw - gMotion.rawYaw);

            gMotion.resetDoneThisStillness = true;
        }
    }
    else
    {
        if (exitedHold)
        {
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.println("========= HOLD RELEASED: MOVEMENT DETECTED =========");
#endif

            gMotion.stillSinceMs = 0;
            gMotion.resetDoneThisStillness = false;
        }
    }

    gMotion.prevHold = hold;
}
static void applyClamps()
{
    const float correctedX = gMotion.rawPosX + gMotion.corrX;
    const float correctedY = gMotion.rawPosY + gMotion.corrY;
    const float correctedZ = gMotion.rawPosZ + gMotion.corrZ;

    const float correctedRoll = wrapAngleDeg(gMotion.rawRoll + gMotion.corrRoll);
    const float correctedPitch = wrapAngleDeg(gMotion.rawPitch + gMotion.corrPitch);
    const float correctedYaw = wrapAngleDeg(gMotion.rawYaw + gMotion.corrYaw);

    const float gyroMagDps = sqrtf(
        gMotion.rawGx * gMotion.rawGx +
        gMotion.rawGy * gMotion.rawGy +
        gMotion.rawGz * gMotion.rawGz);

    const bool holdAnglesToStoredRef =
        (gMotion.stationary || gMotion.zupt) &&
        gMotion.angleHoldRefValid &&
        (gyroMagDps < GYRO_NOISE_THRESH_DPS);

    if (gMotion.stationary || gMotion.zupt)
    {
        // Hold position fixed at last clamped point
        gMotion.corrX = gMotion.clampPosX - gMotion.rawPosX;
        gMotion.corrY = gMotion.clampPosY - gMotion.rawPosY;
        gMotion.corrZ = gMotion.clampPosZ - gMotion.rawPosZ;

        gMotion.clampVelX = 0.0f;
        gMotion.clampVelY = 0.0f;
        gMotion.clampVelZ = 0.0f;

        // Only hold angles if angular motion is just noise
        if (holdAnglesToStoredRef)
        {
#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.println("ANGLE HOLD ACTIVE");
            Serial.printf(
                "APPLY INPUT | gyroMag=%.3f holdValid=%d stationary=%d zupt=%d\n",
                gyroMagDps,
                gMotion.angleHoldRefValid,
                gMotion.stationary,
                gMotion.zupt);

            Serial.printf(
                "ROLL HOLD | raw: %.2f  ref: %.2f  corr: %.2f\n",
                gMotion.rawRoll,
                gMotion.holdRollRef,
                wrapAngleDeg(gMotion.holdRollRef - gMotion.rawRoll));

            Serial.printf(
                "PITCH HOLD | raw: %.2f  ref: %.2f  corr: %.2f\n",
                gMotion.rawPitch,
                gMotion.holdPitchRef,
                wrapAngleDeg(gMotion.holdPitchRef - gMotion.rawPitch));

            Serial.printf(
                "YAW HOLD | raw: %.2f  ref: %.2f  corr: %.2f\n",
                gMotion.rawYaw,
                gMotion.holdYawRef,
                wrapAngleDeg(gMotion.holdYawRef - gMotion.rawYaw));
#endif

            gMotion.corrRoll = wrapAngleDeg(gMotion.holdRollRef - gMotion.rawRoll);
            gMotion.corrPitch = wrapAngleDeg(gMotion.holdPitchRef - gMotion.rawPitch);
            gMotion.corrYaw = wrapAngleDeg(gMotion.holdYawRef - gMotion.rawYaw);

            gMotion.clampRoll = gMotion.holdRollRef;
            gMotion.clampPitch = gMotion.holdPitchRef;
            gMotion.clampYaw = gMotion.holdYawRef;

#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.printf(
                "APPLY OUTPUT | corr=(%.2f, %.2f, %.2f) corrected=(%.2f, %.2f, %.2f) clamp=(%.2f, %.2f, %.2f)\n",
                gMotion.corrRoll,
                gMotion.corrPitch,
                gMotion.corrYaw,
                wrapAngleDeg(gMotion.rawRoll + gMotion.corrRoll),
                wrapAngleDeg(gMotion.rawPitch + gMotion.corrPitch),
                wrapAngleDeg(gMotion.rawYaw + gMotion.corrYaw),
                gMotion.clampRoll,
                gMotion.clampPitch,
                gMotion.clampYaw);
#endif
        }
        else
        {
            // Let orientation move if gyro says this is real motion
            gMotion.corrRoll = 0.0f;
            gMotion.corrPitch = 0.0f;
            gMotion.corrYaw = 0.0f;
            gMotion.clampRoll = correctedRoll;
            gMotion.clampPitch = correctedPitch;
            gMotion.clampYaw = correctedYaw;

#if DEBUG_SERIAL_ENABLE && DEBUG_CLAMP_HOLD
            Serial.printf(
                "ANGLE HOLD BYPASSED | gyroMag=%.3f holdValid=%d corrected=(%.2f, %.2f, %.2f)\n",
                gyroMagDps,
                gMotion.angleHoldRefValid,
                correctedRoll,
                correctedPitch,
                correctedYaw);
#endif
        }

        // imu_zeroVelocity();
    }
    else
    {
        // During motion, let corrected position evolve
        gMotion.clampPosX = correctedX;
        gMotion.clampPosY = correctedY;
        gMotion.clampPosZ = correctedZ;

        gMotion.clampVelX = gMotion.rawVelX;
        gMotion.clampVelY = gMotion.rawVelY;
        gMotion.clampVelZ = gMotion.rawVelZ;

        // During motion, let corrected orientation evolve
        gMotion.corrRoll = 0.0f;
        gMotion.corrPitch = 0.0f;
        gMotion.corrYaw = 0.0f;
        gMotion.clampRoll = correctedRoll;
        gMotion.clampPitch = correctedPitch;
        gMotion.clampYaw = correctedYaw;
    }
}

static void fillLocalPacket(StatePacket &pkt)
{
    pkt.deviceId = SELF_ID;
    pkt.flags = buildFlags();
    pkt.seq = linkPackSeq(gSeq++, gDeadPeerView.directionCode, gDeadPeerView.directionConfidence);
    pkt.timeUs = sharedNowUs();

    pkt.yawDeg = gDeadReckoning.snappedCompareHeadingDeg;
    pkt.initYawDeg = gInitialYawDeg;

    float tx = 0.0f;
    float ty = 0.0f;
    float tz = 0.0f;
    deadReckoningGetSharedPosition(gDeadReckoning, gSharedFrameLocked, gSharedYawOffsetDeg, tx, ty, tz);

    pkt.posX = tx;
    pkt.posY = ty;
    pkt.posZ = tz;

    pkt.speedMps = gDeadReckoning.speedMps;
}

static void computeRelativeDirection()
{
    deadReckoningComputePeerView(
        gDeadReckoning,
        gSharedFrameLocked,
        gSharedYawOffsetDeg,
        gDeadReckoning.snappedCompareHeadingDeg,
        peerPkt.posX,
        peerPkt.posY,
        peerPkt.posZ,
        gDeadPeerView);

    deadReckoningApplyPeerTruth(
        gDeadReckoning,
        gSharedFrameLocked,
        gSharedYawOffsetDeg,
        peerPkt.posX,
        peerPkt.posY,
        peerPkt.posZ,
        peerPkt.yawDeg,
        linkUnpackPeerDirection(peerPkt.seq),
        linkUnpackPeerConfidence(peerPkt.seq),
        (peerPkt.flags & (1 << 2)) != 0,
        (peerPkt.flags & (1 << 4)) != 0,
        gMotion.stationary,
        gDeadReckoning.lockedAfterMove,
        gDeadPeerView);
}

static void printDebug()
{
#if !(DEBUG_SERIAL_ENABLE && (DEBUG_MAIN_STATE || DEBUG_DEAD_RECKONING))
    return;
#else
    Serial.println();
    Serial.println("========== DEBUG ==========");
    Serial.print("SELF_ID=");
    Serial.print(SELF_ID);
    Serial.print(" role=");
#if IS_TIME_MASTER
    Serial.println("TIME_MASTER / POLLER");
#else
    Serial.println("TIME_SLAVE / RESPONDER");
#endif

    Serial.println("-- Time Sync --");
#if IS_TIME_MASTER
    Serial.print("shared_now_us=");
    Serial.println(sharedNowUs());
    Serial.println("locked=yes (master clock)");
#else
    Serial.print("shared_now_us=");
    Serial.println(sharedNowUs());
    Serial.print("offset_us=");
    Serial.println(timeSyncOffsetUs(gTimeSync, micros()));
    Serial.print("rate_ppm=");
    Serial.println(timeSyncRatePpm(gTimeSync), 3);
    Serial.print("locked=");
    Serial.println(timeSyncLocked(gTimeSync) ? "yes" : "no");
#endif

    Serial.println("-- Raw Sensors --");
    Serial.print("accel_raw=(");
    Serial.print(gMotion.rawAx, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawAy, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawAz, 3);
    Serial.println(")");

    Serial.print("gyro_raw=(");
    Serial.print(gMotion.rawGx, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawGy, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawGz, 3);
    Serial.println(")");

    Serial.print("mag_raw=(");
    Serial.print(gMotion.rawMx, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawMy, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawMz, 3);
    Serial.println(")");

    Serial.println("-- Fused IMU --");
    Serial.print("linacc_body=(");
    Serial.print(gMotion.linAxB, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAyB, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAzB, 3);
    Serial.println(")");

    Serial.print("linacc_world=(");
    Serial.print(gMotion.linAxW, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAyW, 3);
    Serial.print(", ");
    Serial.print(gMotion.linAzW, 3);
    Serial.println(")");

    Serial.print("euler_deg=(roll=");
    Serial.print(gMotion.clampRoll, 2);
    Serial.print(", pitch=");
    Serial.print(gMotion.clampPitch, 2);
    Serial.print(", yaw=");
    Serial.print(gMotion.clampYaw, 2);
    Serial.println(")");

    Serial.println("-- Motion Flags --");
    Serial.print("gyro_norm_dps=");
    Serial.println(gMotion.gyroNorm, 3);
    Serial.print("acc_norm_ms2=");
    Serial.println(gMotion.accNorm, 3);
    Serial.print("acc_err_ms2=");
    Serial.println(gMotion.accErr, 3);

    Serial.print("stationary=");
    Serial.println(gMotion.stationary ? "yes" : "no");
    Serial.print("zupt=");
    Serial.println(gMotion.zupt ? "yes" : "no");
    Serial.print("imu_hold=");
    Serial.println(gMotion.imuHoldActive ? "yes" : "no");
    Serial.print("still_count=");
    Serial.println(gMotion.stillCount);
    Serial.print("move_count=");
    Serial.println(gMotion.moveCount);

    Serial.println("-- Shared Frame --");
    Serial.print("initial_yaw_locked=");
    Serial.println(gInitialYawLocked ? "yes" : "no");
    Serial.print("initial_yaw_deg=");
    Serial.println(gInitialYawDeg, 2);
    Serial.print("shared_frame_locked=");
    Serial.println(gSharedFrameLocked ? "yes" : "no");
    Serial.print("shared_yaw_offset_deg=");
    Serial.println(gSharedYawOffsetDeg, 2);

    Serial.println("-- Position / Velocity --");
    Serial.print("raw_pos=(");
    Serial.print(gMotion.rawPosX, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawPosY, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawPosZ, 3);
    Serial.println(")");

    Serial.print("raw_vel=(");
    Serial.print(gMotion.rawVelX, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawVelY, 3);
    Serial.print(", ");
    Serial.print(gMotion.rawVelZ, 3);
    Serial.println(")");

    Serial.print("corr_offset=(");
    Serial.print(gMotion.corrX, 3);
    Serial.print(", ");
    Serial.print(gMotion.corrY, 3);
    Serial.print(", ");
    Serial.print(gMotion.corrZ, 3);
    Serial.println(")");

    Serial.print("clamped_pos=(");
    Serial.print(gMotion.clampPosX, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampPosY, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampPosZ, 3);
    Serial.println(")");

    Serial.print("clamped_vel=(");
    Serial.print(gMotion.clampVelX, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampVelY, 3);
    Serial.print(", ");
    Serial.print(gMotion.clampVelZ, 3);
    Serial.println(")");

    Serial.println("-- Dead Reckoning --");
    Serial.print("accel_mag_ms2=");
    Serial.println(gDeadReckoning.accelMss, 3);
    Serial.print("step_m=");
    Serial.println(gDeadReckoning.stepMeters, 4);
    Serial.print("gyro_heading_deg=");
    Serial.println(gDeadReckoning.gyroHeadingDeg, 2);
    Serial.print("quantized_move_heading_deg=");
    Serial.println(gDeadReckoning.quantizedMoveHeadingDeg, 2);
    Serial.print("snapped_roll_deg=");
    Serial.println(gDeadReckoning.snappedCompareRollDeg, 2);
    Serial.print("snapped_heading_deg=");
    Serial.println(gDeadReckoning.snappedCompareHeadingDeg, 2);
    Serial.print("snapped_facing=");
    Serial.println(gDeadReckoning.snappedFacingName);
    Serial.print("locked_after_move=");
    Serial.println(gDeadReckoning.lockedAfterMove ? "yes" : "no");
    Serial.print("dead_reckon_speed_mps=");
    Serial.println(gDeadReckoning.speedMps, 3);
    Serial.print("dead_reckon_pos=(");
    Serial.print(gDeadReckoning.posX, 3);
    Serial.print(", ");
    Serial.print(gDeadReckoning.posY, 3);
    Serial.print(", ");
    Serial.print(gDeadReckoning.posZ, 3);
    Serial.println(")");
    Serial.print("dead_reckon_vel=(");
    Serial.print(gDeadReckoning.velX, 3);
    Serial.print(", ");
    Serial.print(gDeadReckoning.velY, 3);
    Serial.print(", ");
    Serial.print(gDeadReckoning.velZ, 3);
    Serial.println(")");

    Serial.println("-- Peer State --");
    Serial.print("peer_id=");
    Serial.println(peerPkt.deviceId);
    Serial.print("peer_packet_stale=");
    Serial.println(gPeerPacketStale ? "yes" : "no");
    Serial.print("peer_t_us=");
    Serial.println(peerPkt.timeUs);
    Serial.print("peer_seq_counter=");
    Serial.println(linkUnpackCounter(peerPkt.seq));
    Serial.print("peer_pos=(");
    Serial.print(peerPkt.posX, 3);
    Serial.print(", ");
    Serial.print(peerPkt.posY, 3);
    Serial.print(", ");
    Serial.print(peerPkt.posZ, 3);
    Serial.println(")");
    Serial.print("peer_yaw_deg=");
    Serial.println(peerPkt.yawDeg, 2);
    Serial.print("peer_init_yaw_deg=");
    Serial.println(peerPkt.initYawDeg, 2);
    Serial.print("peer_speed_mps=");
    Serial.println(peerPkt.speedMps, 3);
    Serial.print("peer_locked_after_move=");
    Serial.println(((peerPkt.flags & (1 << 4)) != 0) ? "yes" : "no");
    Serial.print("peer_reports_me_as=");
    Serial.println(deadReckoningDirectionName(linkUnpackPeerDirection(peerPkt.seq)));
    Serial.print("peer_report_confidence=");
    Serial.println(linkUnpackPeerConfidence(peerPkt.seq));

    Serial.println("-- Relative To Peer --");
    Serial.print("delta_xyz=(");
    Serial.print(gDeadPeerView.dx, 3);
    Serial.print(", ");
    Serial.print(gDeadPeerView.dy, 3);
    Serial.print(", ");
    Serial.print(gDeadPeerView.dz, 3);
    Serial.println(")");
    Serial.print("distance_m=");
    Serial.println(gDeadPeerView.distance, 3);
    Serial.print("bearing_world_deg=");
    Serial.println(gDeadPeerView.bearingWorldDeg, 2);
    Serial.print("bearing_local_deg=");
    Serial.println(gDeadPeerView.bearingLocalDeg, 2);
    Serial.print("peer_close=");
    Serial.println(gDeadPeerView.isClose ? "yes" : "no");
    Serial.print("peer_direction=");
    Serial.println(deadReckoningDirectionName(gDeadPeerView.directionCode));
    Serial.print("peer_direction_confidence=");
    Serial.println(gDeadPeerView.directionConfidence);
    Serial.print("peer_truth_applied=");
    Serial.println(gDeadReckoning.peerTruthApplied ? "yes" : "no");

    Serial.print("euler_raw_deg=(roll=");
    Serial.print(gMotion.rawRoll, 2);
    Serial.print(", pitch=");
    Serial.print(gMotion.rawPitch, 2);
    Serial.print(", yaw=");
    Serial.print(gMotion.rawYaw, 2);
    Serial.println(")");
    Serial.print("euler_raw_facing=");
    Serial.println(gDeadReckoning.snappedFacingName);

    Serial.print("euler_clamped_deg=(roll=");
    Serial.print(gMotion.clampRoll, 2);
    Serial.print(", pitch=");
    Serial.print(gMotion.clampPitch, 2);
    Serial.print(", yaw=");
    Serial.print(gMotion.clampYaw, 2);
    Serial.println(")");

    Serial.println("===========================");
    Serial.println();
#endif
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    SPIFFS.begin(true);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);

    SPI.begin(18, 19, 23, 5);

    if (!imu_begin())
    {
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
        Serial.println("imu_begin() failed");
#endif
        while (1)
        {
        }
    }

    if (!radio.begin())
    {
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
        Serial.println("radio.begin() failed");
#endif
        while (1)
        {
        }
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setCRCLength(RF24_CRC_16);

    timeSyncBegin(gTimeSync, IS_TIME_MASTER != 0);

#if IS_POLLER
    linkBegin(radio, LINK_ROLE_POLLER, SELF_ID);
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
    Serial.print("Started as TIME MASTER / POLLER, SELF_ID=");
    Serial.println(SELF_ID);
#endif
#else
    linkBegin(radio, LINK_ROLE_RESPONDER, SELF_ID);
#if DEBUG_SERIAL_ENABLE && DEBUG_BOOT_LOGS
    Serial.print("Started as TIME SLAVE / RESPONDER, SELF_ID=");
    Serial.println(SELF_ID);
#endif
#endif

    gLastDebugMs = millis();
    gLastTelemetryLogMs = millis();
    gLastWifiCheckMs = millis();
    deadReckoningInit(gDeadReckoning, micros());
}

void loop()
{
    imu_update();
    captureImuState();
    detectMotionFlags();
    updateStationaryHold();
    applyClamps();

    DeadReckoningInput drInput{};
    drInput.linAccelBodyX = gMotion.linAxB;
    drInput.linAccelBodyY = gMotion.linAyB;
    drInput.linAccelBodyZ = gMotion.linAzB;
    drInput.gyroHeadingDeg = gMotion.clampYaw;
    drInput.compareRollDeg = gMotion.rawRoll;
    drInput.stationary = gMotion.stationary;
    drInput.zupt = gMotion.zupt;
    deadReckoningUpdate(gDeadReckoning, drInput, micros());
    maybeLockInitialYaw();

    fillLocalPacket(localPkt);
    linkSetLocalState(localPkt);

#if IS_POLLER
    if (linkExchange(radio, peerPkt))
    {
        gHavePeerPacket = true;
        gPeerPacketStale = false;
        gLastPeerPacketMs = millis();
        computeRelativeDirection();
    }
#else
    if (linkPollResponder(radio, peerPkt))
    {
        const uint32_t localRxUs = micros();
        gHavePeerPacket = true;
        gPeerPacketStale = false;
        gLastPeerPacketMs = millis();

        if (peerPkt.deviceId == 1)
            timeSyncObserveMaster(gTimeSync, localRxUs, peerPkt.timeUs);

        maybeLockSharedFrameFromPeer();
        computeRelativeDirection();
    }
#endif

    if (gHavePeerPacket && (millis() - gLastPeerPacketMs > PEER_PACKET_STALE_MS))
    {
        gPeerPacketStale = true;
        computeRelativeDirection();
    }

#if IS_POLLER
    // Master always defines the shared frame
    if (gInitialYawLocked && !gSharedFrameLocked)
    {
        gSharedYawOffsetDeg = 0.0f;
        gSharedFrameLocked = true;
    }
#endif

    if (millis() - gLastDebugMs >= 1000)
    {
        gLastDebugMs = millis();
        printDebug();
    }

    maybeLogTelemetry();
    maybeUploadTelemetry();

#if IS_POLLER
    delay(20);
#else
    delay(2);
#endif
}
