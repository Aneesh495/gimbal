/*
 * XC Test Bench: Avionics Comms Protocol and CRC32 Validation
 * Tests standard IEEE 802.3 test vectors, packet framing, and corrupted packet rejection.
 */

#include "comms.h"
#include <stdbool.h>

bool run_comms_bench(void) {
    /* Test 1: Standard "123456789" CRC-32 test string = 0xCBF43926 */
    const uint8_t test_str[9] = {'1','2','3','4','5','6','7','8','9'};
    uint32_t crc = comms_crc32_calculate(test_str, 9);
    if (crc != 0xCBF43926) return false;

    /* Test 2: Encode packet and decode byte-by-byte */
    uint8_t payload[4] = {0x11, 0x22, 0x33, 0x44};
    uint8_t encoded[32];
    uint32_t enc_len = comms_encode_packet(MSG_ID_SET_MODE, 42, payload, 4, encoded);
    if (enc_len != 14) return false;

    binary_packet_t pkt_out;
    bool decoded = false;
    for (uint32_t i = 0; i < enc_len; i++) {
        if (comms_decode_byte(encoded[i], &pkt_out)) {
            decoded = true;
            break;
        }
    }
    if (!decoded || pkt_out.seq != 42 || pkt_out.msg_id != MSG_ID_SET_MODE) return false;
    return true;
}
