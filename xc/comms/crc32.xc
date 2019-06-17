/*
 * Gimbal Hardware-Accelerated CRC-32 IEEE 802.3 Checksum
 * Fast 8-bit table-driven and bitwise computation for CSP packets.
 */

#include "comms.h"

#define CRC32_POLYNOMIAL 0xEDB88320

static uint32_t crc32_table[256];
static bool table_initialized = false;

static void crc32_init_table(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++) {
            c = (c & 1) ? (CRC32_POLYNOMIAL ^ (c >> 1)) : (c >> 1);
        }
        crc32_table[i] = c;
    }
    table_initialized = true;
}

uint32_t comms_crc32_update(uint32_t current_crc, uint8_t byte) {
    if (!table_initialized) {
        crc32_init_table();
    }
    uint8_t idx = (uint8_t)(current_crc ^ byte);
    return crc32_table[idx] ^ (current_crc >> 8);
}

uint32_t comms_crc32_calculate(const uint8_t *data, uint32_t length) {
    if (!data) return 0;
    if (!table_initialized) {
        crc32_init_table();
    }
    
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++) {
        uint8_t idx = (uint8_t)(crc ^ data[i]);
        crc = crc32_table[idx] ^ (crc >> 8);
    }
    return ~crc;
}
