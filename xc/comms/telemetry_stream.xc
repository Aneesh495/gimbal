/*
 * Gimbal High-Rate Packed Binary Telemetry Downlink Streamer
 * Serializes attitude quaternion, angular rates, motor currents, and temperatures.
 */

#include "comms.h"

static uint16_t telemetry_seq = 0;

uint32_t comms_pack_telemetry(const nav_state_t *nav, const foc_state_t foc[3], uint8_t *dest) {
    if (!dest) return 0;
    
    uint8_t payload[48];
    int32_t *p32 = (int32_t*)payload;
    
    /* Attitude quaternion (16 bytes) */
    p32[0] = nav ? nav->q_inertial_to_body.w : FIXED_Q16_ONE;
    p32[1] = nav ? nav->q_inertial_to_body.x : 0;
    p32[2] = nav ? nav->q_inertial_to_body.y : 0;
    p32[3] = nav ? nav->q_inertial_to_body.z : 0;
    
    /* Angular rates (12 bytes) */
    p32[4] = nav ? nav->rate_body_rad_s.x : 0;
    p32[5] = nav ? nav->rate_body_rad_s.y : 0;
    p32[6] = nav ? nav->rate_body_rad_s.z : 0;
    
    /* Iq current for Pan, Tilt, Roll (12 bytes) */
    p32[7] = foc ? foc[0].i_q : 0;
    p32[8] = foc ? foc[1].i_q : 0;
    p32[9] = foc ? foc[2].i_q : 0;
    
    /* Estimated disturbance torque (8 bytes: pan & tilt) */
    p32[10] = nav ? nav->disturbance_torque_nm.x : 0;
    p32[11] = nav ? nav->disturbance_torque_nm.y : 0;
    
    telemetry_seq++;
    return comms_encode_packet(MSG_ID_ATTITUDE_TLM, telemetry_seq, payload, 48, dest);
}
