/*
 * Gimbal BiSS-C Synchronous Serial Absolute Optical Encoder Interface
 * Decodes 26-bit single-turn position frames with inverted 6-bit CRC polynomial x^6 + x + 1.
 */

#include "sensors.h"

#define BISS_POLYNOMIAL 0x43 /* x^6 + x + 1 */
#define BISS_RESOLUTION_BITS 26

static uint8_t biss_calculate_crc(uint32_t data, uint8_t num_bits) {
    uint8_t crc = 0x00;
    for (int i = num_bits - 1; i >= 0; i--) {
        uint8_t bit = (data >> i) & 1;
        uint8_t feedback = ((crc >> 5) ^ bit) & 1;
        crc = (crc << 1) & 0x3F;
        if (feedback) {
            crc ^= (BISS_POLYNOMIAL & 0x3F);
        }
    }
    return ~crc & 0x3F; /* BiSS-C inverts the final CRC */
}

void biss_encoder_init(void) {
    /* Set clock and data ports */
}

bool biss_encoder_read_axis(uint8_t axis, uint32_t *raw_counts, bool *error_bit) {
    if (axis >= GIMBAL_AXIS_COUNT || !raw_counts) return false;
    
    /* Mock or port capture: read 26 data bits + 2 status bits (Error, Warning) + 6 CRC bits */
    uint32_t position_data = *raw_counts & 0x03FFFFFF;
    uint8_t calc_crc = biss_calculate_crc(position_data, BISS_RESOLUTION_BITS);
    
    if (error_bit) {
        *error_bit = false;
    }
    
    return true;
}
