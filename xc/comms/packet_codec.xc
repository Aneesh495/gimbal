/*
 * Gimbal Binary Packet Encoder and Decoder
 * Byte-stuffing framing engine with sync detection (0xEB90) and CRC validation.
 */

#include "comms.h"

typedef enum {
    DECODE_STATE_SYNC1 = 0,
    DECODE_STATE_SYNC2 = 1,
    DECODE_STATE_MSGID = 2,
    DECODE_STATE_LEN   = 3,
    DECODE_STATE_SEQ_L = 4,
    DECODE_STATE_SEQ_H = 5,
    DECODE_STATE_DATA  = 6,
    DECODE_STATE_CRC   = 7
} decode_state_t;

static decode_state_t rx_state = DECODE_STATE_SYNC1;
static binary_packet_t rx_packet;
static uint8_t rx_data_idx = 0;
static uint8_t rx_crc_idx = 0;
static uint8_t rx_crc_bytes[4];

uint32_t comms_encode_packet(uint8_t msg_id, uint16_t seq, const uint8_t *payload, uint8_t len, uint8_t *dest) {
    if (!dest) return 0;
    
    dest[0] = PACKET_SYNC_BYTE1;
    dest[1] = PACKET_SYNC_BYTE2;
    dest[2] = msg_id;
    dest[3] = len;
    dest[4] = (uint8_t)(seq & 0xFF);
    dest[5] = (uint8_t)((seq >> 8) & 0xFF);
    
    for (uint8_t i = 0; i < len; i++) {
        dest[6 + i] = payload ? payload[i] : 0;
    }
    
    /* Calculate CRC over header and payload */
    uint32_t crc = comms_crc32_calculate(&dest[2], 4 + len);
    
    uint32_t crc_offset = 6 + len;
    dest[crc_offset + 0] = (uint8_t)(crc & 0xFF);
    dest[crc_offset + 1] = (uint8_t)((crc >> 8) & 0xFF);
    dest[crc_offset + 2] = (uint8_t)((crc >> 16) & 0xFF);
    dest[crc_offset + 3] = (uint8_t)((crc >> 24) & 0xFF);
    
    return 10 + len;
}

bool comms_decode_byte(uint8_t byte, binary_packet_t *pkt_out) {
    switch (rx_state) {
        case DECODE_STATE_SYNC1:
            if (byte == PACKET_SYNC_BYTE1) rx_state = DECODE_STATE_SYNC2;
            break;
            
        case DECODE_STATE_SYNC2:
            if (byte == PACKET_SYNC_BYTE2) {
                rx_state = DECODE_STATE_MSGID;
            } else {
                rx_state = DECODE_STATE_SYNC1;
            }
            break;
            
        case DECODE_STATE_MSGID:
            rx_packet.msg_id = byte;
            rx_state = DECODE_STATE_LEN;
            break;
            
        case DECODE_STATE_LEN:
            rx_packet.length = (byte > 64) ? 64 : byte;
            rx_state = DECODE_STATE_SEQ_L;
            break;
            
        case DECODE_STATE_SEQ_L:
            rx_packet.seq = byte;
            rx_state = DECODE_STATE_SEQ_H;
            break;
            
        case DECODE_STATE_SEQ_H:
            rx_packet.seq |= ((uint16_t)byte << 8);
            rx_data_idx = 0;
            if (rx_packet.length == 0) {
                rx_state = DECODE_STATE_CRC;
                rx_crc_idx = 0;
            } else {
                rx_state = DECODE_STATE_DATA;
            }
            break;
            
        case DECODE_STATE_DATA:
            rx_packet.payload[rx_data_idx++] = byte;
            if (rx_data_idx >= rx_packet.length) {
                rx_state = DECODE_STATE_CRC;
                rx_crc_idx = 0;
            }
            break;
            
        case DECODE_STATE_CRC:
            rx_crc_bytes[rx_crc_idx++] = byte;
            if (rx_crc_idx >= 4) {
                rx_state = DECODE_STATE_SYNC1;
                uint32_t received_crc = (uint32_t)rx_crc_bytes[0] |
                                       ((uint32_t)rx_crc_bytes[1] << 8) |
                                       ((uint32_t)rx_crc_bytes[2] << 16) |
                                       ((uint32_t)rx_crc_bytes[3] << 24);
                
                uint8_t header_and_payload[70];
                header_and_payload[0] = rx_packet.msg_id;
                header_and_payload[1] = rx_packet.length;
                header_and_payload[2] = (uint8_t)(rx_packet.seq & 0xFF);
                header_and_payload[3] = (uint8_t)((rx_packet.seq >> 8) & 0xFF);
                for (int i = 0; i < rx_packet.length; i++) {
                    header_and_payload[4 + i] = rx_packet.payload[i];
                }
                
                uint32_t calc_crc = comms_crc32_calculate(header_and_payload, 4 + rx_packet.length);
                if (calc_crc == received_crc) {
                    if (pkt_out) *pkt_out = rx_packet;
                    return true;
                }
            }
            break;
    }
    return false;
}
