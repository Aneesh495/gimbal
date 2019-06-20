/*
 * Gimbal Avionics Command Dispatcher
 * Parses flight computer command packets, validates parameter boundaries, and sets operating modes.
 */

#include "comms.h"

void comms_handle_command(const binary_packet_t *pkt, gimbal_cmd_t *cmd_out) {
    if (!pkt || !cmd_out) return;
    
    cmd_out->cmd_sequence = pkt->seq;
    
    switch (pkt->msg_id) {
        case MSG_ID_SET_MODE:
            if (pkt->length >= 1) {
                uint8_t mode = pkt->payload[0];
                if (mode <= GIMBAL_MODE_FAULT) {
                    cmd_out->mode = (gimbal_mode_t)mode;
                }
            }
            break;
            
        case MSG_ID_SET_RATE_CMD:
            if (pkt->length >= 12) {
                cmd_out->mode = GIMBAL_MODE_RATE_HOLD;
                int32_t *data = (int32_t*)pkt->payload;
                cmd_out->target_rate_cmd.x = (q16_t)data[0];
                cmd_out->target_rate_cmd.y = (q16_t)data[1];
                cmd_out->target_rate_cmd.z = (q16_t)data[2];
            }
            break;
            
        case MSG_ID_SET_ANGLE_CMD:
            if (pkt->length >= 12) {
                cmd_out->mode = GIMBAL_MODE_ATT_HOLD;
                int32_t *data = (int32_t*)pkt->payload;
                cmd_out->target_angle_cmd.x = (q16_t)data[0];
                cmd_out->target_angle_cmd.y = (q16_t)data[1];
                cmd_out->target_angle_cmd.z = (q16_t)data[2];
            }
            break;
            
        case MSG_ID_SET_GEO_CMD:
            if (pkt->length >= 12) {
                cmd_out->mode = GIMBAL_MODE_GEO_POINT;
                int32_t *data = (int32_t*)pkt->payload;
                cmd_out->geo_lat_lon_alt.x = (q16_t)data[0];
                cmd_out->geo_lat_lon_alt.y = (q16_t)data[1];
                cmd_out->geo_lat_lon_alt.z = (q16_t)data[2];
            }
            break;
            
        default:
            break;
    }
}
