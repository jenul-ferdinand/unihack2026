#pragma once

#include <Arduino.h>
#include <RF24.h>

enum LinkRole
{
    LINK_MASTER = 0,
    LINK_SLAVE  = 1
};

struct __attribute__((packed)) ImuPacket
{
    uint32_t t_ms;

    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
    float raw_mx, raw_my, raw_mz;

    float lin_ax, lin_ay, lin_az;
    float world_ax, world_ay, world_az;

    float q0, q1, q2, q3;
    float roll, pitch, yaw;

    float px, py, pz;
    float vx, vy, vz;
};

// RF24 max payload is 32 bytes.
// This frame is exactly 32 bytes.
struct __attribute__((packed)) RadioFrame
{
    uint8_t magic;         // 0xA5
    uint8_t version;       // 1
    uint8_t seq;           // packet sequence number
    uint8_t part_index;    // which fragment this is
    uint8_t part_count;    // total fragment count
    uint8_t payload_len;   // bytes valid in payload[]
    uint16_t reserved;     // padding / future use

    uint8_t payload[24];   // 24 bytes of actual chunk data
};

static_assert(sizeof(RadioFrame) == 32, "RadioFrame must be 32 bytes");

struct PacketAssembler
{
    uint8_t seq = 0xFF;
    uint8_t expected_parts = 0;
    uint8_t received_mask = 0;   // enough for up to 8 parts
    uint8_t buffer[sizeof(ImuPacket)] = {0};
};

struct LinkState
{
    ImuPacket my_packet = {};
    ImuPacket peer_packet = {};

    bool connected = false;
    bool got_peer_packet = false;

    uint32_t tx_count = 0;
    uint32_t rx_count = 0;
    uint32_t tx_fail_count = 0;
    uint32_t last_success_ms = 0;

    // tx fragmentation state
    uint8_t tx_seq = 0;
    uint8_t tx_part_index = 0;
    uint8_t tx_buffer[sizeof(ImuPacket)] = {0};

    // rx reassembly state
    PacketAssembler rx_assembler;
};

bool rf24_link_begin(RF24 &radio, LinkRole role);
void rf24_link_update_local_packet(LinkState &state);

bool rf24_link_master_step(RF24 &radio, LinkState &state);
bool rf24_link_slave_step(RF24 &radio, LinkState &state);

void rf24_link_print_packet(const char *label, const ImuPacket &p);
void rf24_link_print_status(const char *label, const LinkState &state);