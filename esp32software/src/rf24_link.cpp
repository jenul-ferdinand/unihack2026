#include "rf24_link.h"
#include "imu.h"

static const byte ADDR_MASTER[6] = "NODE1";
static const byte ADDR_SLAVE[6]  = "NODE2";

static constexpr uint8_t FRAME_MAGIC = 0xA5;
static constexpr uint8_t FRAME_VERSION = 1;
static constexpr uint8_t FRAME_PAYLOAD_BYTES = 24;
static constexpr uint8_t IMU_PACKET_BYTES = sizeof(ImuPacket);
static constexpr uint8_t IMU_PART_COUNT =
    (IMU_PACKET_BYTES + FRAME_PAYLOAD_BYTES - 1) / FRAME_PAYLOAD_BYTES;

static_assert(IMU_PART_COUNT <= 8, "received_mask only supports up to 8 parts");

static void serialize_local_packet(LinkState &state)
{
    memcpy(state.tx_buffer, &state.my_packet, sizeof(ImuPacket));
}

static void begin_new_tx_packet(LinkState &state)
{
    serialize_local_packet(state);
    state.tx_seq++;
    state.tx_part_index = 0;
}

static void build_tx_frame(LinkState &state, RadioFrame &frame)
{
    if (state.tx_part_index >= IMU_PART_COUNT)
    {
        begin_new_tx_packet(state);
    }

    const uint16_t offset = state.tx_part_index * FRAME_PAYLOAD_BYTES;
    const uint16_t remain = IMU_PACKET_BYTES - offset;
    const uint8_t chunk_len = (remain >= FRAME_PAYLOAD_BYTES) ? FRAME_PAYLOAD_BYTES : remain;

    frame.magic = FRAME_MAGIC;
    frame.version = FRAME_VERSION;
    frame.seq = state.tx_seq;
    frame.part_index = state.tx_part_index;
    frame.part_count = IMU_PART_COUNT;
    frame.payload_len = chunk_len;
    frame.reserved = 0;

    memset(frame.payload, 0, sizeof(frame.payload));
    memcpy(frame.payload, state.tx_buffer + offset, chunk_len);

    state.tx_part_index++;
    if (state.tx_part_index >= IMU_PART_COUNT)
    {
        // next time we start a fresh packet snapshot
        // do not increment seq yet, that happens when next packet begins
        state.tx_part_index = IMU_PART_COUNT;
    }
}

static void maybe_roll_tx_after_full_cycle(LinkState &state)
{
    if (state.tx_part_index >= IMU_PART_COUNT)
    {
        begin_new_tx_packet(state);
    }
}

static bool process_rx_frame(LinkState &state, const RadioFrame &frame)
{
    if (frame.magic != FRAME_MAGIC) return false;
    if (frame.version != FRAME_VERSION) return false;
    if (frame.part_count != IMU_PART_COUNT) return false;
    if (frame.part_index >= frame.part_count) return false;
    if (frame.payload_len > FRAME_PAYLOAD_BYTES) return false;

    const uint16_t offset = frame.part_index * FRAME_PAYLOAD_BYTES;
    if (offset + frame.payload_len > IMU_PACKET_BYTES) return false;

    PacketAssembler &a = state.rx_assembler;

    // new sequence, start fresh
    if (a.seq != frame.seq)
    {
        a.seq = frame.seq;
        a.expected_parts = frame.part_count;
        a.received_mask = 0;
        memset(a.buffer, 0, sizeof(a.buffer));
    }

    memcpy(a.buffer + offset, frame.payload, frame.payload_len);
    a.received_mask |= (1u << frame.part_index);

    const uint8_t full_mask = (1u << frame.part_count) - 1u;
    if (a.received_mask == full_mask)
    {
        memcpy(&state.peer_packet, a.buffer, sizeof(ImuPacket));
        state.got_peer_packet = true;
        state.connected = true;
        state.rx_count++;
        state.last_success_ms = millis();

        // force next seq to start clean
        a.seq = 0xFF;
        a.expected_parts = 0;
        a.received_mask = 0;
        memset(a.buffer, 0, sizeof(a.buffer));
        return true;
    }

    return false;
}

bool rf24_link_begin(RF24 &radio, LinkRole role)
{
    if (!radio.begin())
    {
        Serial.println("RF24 begin failed");
        return false;
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.setRetries(5, 15);

    radio.flush_tx();
    radio.flush_rx();

    if (role == LINK_MASTER)
    {
        // Master writes to slave, listens on master address
        radio.openWritingPipe(ADDR_SLAVE);
        radio.openReadingPipe(1, ADDR_MASTER);
        Serial.println("RF24 link configured as MASTER");
    }
    else
    {
        // Slave receives on slave address, ack payload goes back automatically
        radio.openWritingPipe(ADDR_MASTER);
        radio.openReadingPipe(1, ADDR_SLAVE);
        Serial.println("RF24 link configured as SLAVE");
    }

    radio.startListening();
    delay(5);
    return true;
}

void rf24_link_update_local_packet(LinkState &state)
{
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
    float raw_mx, raw_my, raw_mz;

    float lin_ax, lin_ay, lin_az;
    float world_ax, world_ay, world_az;

    float q0, q1, q2, q3;
    float roll, pitch, yaw;

    float px, py, pz;
    float vx, vy, vz;

    imu_getRawAccel(raw_ax, raw_ay, raw_az);
    imu_getRawGyro(raw_gx, raw_gy, raw_gz);
    imu_getRawMag(raw_mx, raw_my, raw_mz);

    imu_getLinearAccel(lin_ax, lin_ay, lin_az);
    imu_getLinearAccelWorld(world_ax, world_ay, world_az);

    imu_getQuat(q0, q1, q2, q3);
    imu_getEuler(roll, pitch, yaw);

    imu_getPosition(px, py, pz);
    imu_getVelocity(vx, vy, vz);

    ImuPacket &p = state.my_packet;
    p.t_ms = millis();

    p.raw_ax = raw_ax; p.raw_ay = raw_ay; p.raw_az = raw_az;
    p.raw_gx = raw_gx; p.raw_gy = raw_gy; p.raw_gz = raw_gz;
    p.raw_mx = raw_mx; p.raw_my = raw_my; p.raw_mz = raw_mz;

    p.lin_ax = lin_ax; p.lin_ay = lin_ay; p.lin_az = lin_az;
    p.world_ax = world_ax; p.world_ay = world_ay; p.world_az = world_az;

    p.q0 = q0; p.q1 = q1; p.q2 = q2; p.q3 = q3;
    p.roll = roll; p.pitch = pitch; p.yaw = yaw;

    p.px = px; p.py = py; p.pz = pz;
    p.vx = vx; p.vy = vy; p.vz = vz;
}

bool rf24_link_master_step(RF24 &radio, LinkState &state)
{
    state.got_peer_packet = false;

    // if we just finished a packet last cycle, begin a new snapshot now
    maybe_roll_tx_after_full_cycle(state);

    RadioFrame out = {};
    build_tx_frame(state, out);

    radio.stopListening();
    bool ok = radio.write(&out, sizeof(out));
    radio.startListening();

    state.tx_count++;

    if (!ok)
    {
        state.tx_fail_count++;
        return false;
    }

    if (radio.isAckPayloadAvailable())
    {
        uint8_t len = radio.getDynamicPayloadSize();
        if (len == sizeof(RadioFrame))
        {
            RadioFrame in = {};
            radio.read(&in, sizeof(in));
            process_rx_frame(state, in);
        }
        else
        {
            radio.flush_rx();
        }
    }

    if (millis() - state.last_success_ms > 1000)
    {
        state.connected = false;
    }

    return state.got_peer_packet;
}

bool rf24_link_slave_step(RF24 &radio, LinkState &state)
{
    state.got_peer_packet = false;

    bool processed_any = false;

    while (radio.available())
    {
        uint8_t len = radio.getDynamicPayloadSize();
        if (len != sizeof(RadioFrame))
        {
            radio.flush_rx();
            return false;
        }

        RadioFrame in = {};
        radio.read(&in, sizeof(in));
        processed_any = true;

        process_rx_frame(state, in);

        // queue next outgoing fragment as ACK payload
        maybe_roll_tx_after_full_cycle(state);

        RadioFrame ack = {};
        build_tx_frame(state, ack);
        radio.writeAckPayload(1, &ack, sizeof(ack));
    }

    if (millis() - state.last_success_ms > 1000)
    {
        state.connected = false;
    }

    return processed_any;
}

void rf24_link_print_packet(const char *label, const ImuPacket &p)
{
    Serial.print(label);
    Serial.print(" t=");
    Serial.print(p.t_ms);

    Serial.print(" rawA=(");
    Serial.print(p.raw_ax, 3); Serial.print(", ");
    Serial.print(p.raw_ay, 3); Serial.print(", ");
    Serial.print(p.raw_az, 3); Serial.print(")");

    Serial.print(" rawG=(");
    Serial.print(p.raw_gx, 3); Serial.print(", ");
    Serial.print(p.raw_gy, 3); Serial.print(", ");
    Serial.print(p.raw_gz, 3); Serial.print(")");

    Serial.print(" rawM=(");
    Serial.print(p.raw_mx, 3); Serial.print(", ");
    Serial.print(p.raw_my, 3); Serial.print(", ");
    Serial.print(p.raw_mz, 3); Serial.print(")");

    Serial.print(" linB=(");
    Serial.print(p.lin_ax, 3); Serial.print(", ");
    Serial.print(p.lin_ay, 3); Serial.print(", ");
    Serial.print(p.lin_az, 3); Serial.print(")");

    Serial.print(" linW=(");
    Serial.print(p.world_ax, 3); Serial.print(", ");
    Serial.print(p.world_ay, 3); Serial.print(", ");
    Serial.print(p.world_az, 3); Serial.print(")");

    Serial.print(" rpy=(");
    Serial.print(p.roll, 2); Serial.print(", ");
    Serial.print(p.pitch, 2); Serial.print(", ");
    Serial.print(p.yaw, 2); Serial.print(")");

    Serial.print(" pos=(");
    Serial.print(p.px, 3); Serial.print(", ");
    Serial.print(p.py, 3); Serial.print(", ");
    Serial.print(p.pz, 3); Serial.print(")");

    Serial.print(" vel=(");
    Serial.print(p.vx, 3); Serial.print(", ");
    Serial.print(p.vy, 3); Serial.print(", ");
    Serial.print(p.vz, 3); Serial.println(")");
}

void rf24_link_print_status(const char *label, const LinkState &state)
{
    Serial.print(label);
    Serial.print(" connected=");
    Serial.print(state.connected ? "yes" : "no");
    Serial.print(" got_peer=");
    Serial.print(state.got_peer_packet ? "yes" : "no");
    Serial.print(" tx=");
    Serial.print(state.tx_count);
    Serial.print(" rx=");
    Serial.print(state.rx_count);
    Serial.print(" tx_fail=");
    Serial.print(state.tx_fail_count);
    Serial.print(" last_ok_ms=");
    Serial.println(state.last_success_ms);
}