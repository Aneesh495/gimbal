#ifndef GIMBAL_COMMS_H
#define GIMBAL_COMMS_H

#include "gimbal_types.h"

#define PACKET_SYNC_BYTE1 0xEB
#define PACKET_SYNC_BYTE2 0x90

#define MSG_ID_HEARTBEAT     0x01
#define MSG_ID_ATTITUDE_TLM  0x10
#define MSG_ID_MOTOR_TLM     0x11
#define MSG_ID_SENSOR_TLM    0x12
#define MSG_ID_SET_MODE      0x20
#define MSG_ID_SET_RATE_CMD  0x21
#define MSG_ID_SET_ANGLE_CMD 0x22
#define MSG_ID_SET_GEO_CMD   0x23

typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t msg_id;
    uint8_t length;
    uint16_t seq;
    uint8_t payload[64];
    uint32_t checksum;
} binary_packet_t;

uint32_t comms_crc32_calculate(const uint8_t *data, uint32_t length);
uint32_t comms_crc32_update(uint32_t current_crc, uint8_t byte);

uint32_t comms_encode_packet(uint8_t msg_id, uint16_t seq, const uint8_t *payload, uint8_t len, uint8_t *dest);
bool comms_decode_byte(uint8_t byte, binary_packet_t *pkt_out);

void comms_handle_command(const binary_packet_t *pkt, gimbal_cmd_t *cmd_out);
uint32_t comms_pack_telemetry(const nav_state_t *nav, const foc_state_t foc[3], uint8_t *dest);

#endif
