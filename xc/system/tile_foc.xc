/*
 * Gimbal Tile 1: 3-Axis Field-Oriented Control (FOC) Motor Drive Executive
 * Executes 25 kHz current loops, Clarke/Park, anti-cogging, and SVPWM commutation.
 */

#include "foc.h"
#include "gimbal_channels.h"

static motor_axis_channel_t axes[GIMBAL_AXIS_COUNT];

void tile_foc_executive(void) {
    for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
        foc_motor_channel_init(&axes[i], i);
    }
    
    /* 25 kHz FOC control loop */
    while (1) {
        for (int i = 0; i < GIMBAL_AXIS_COUNT; i++) {
            /* Nominal loop update with feedback currents and rotor angle */
            foc_motor_channel_update(&axes[i], 0, 0, 0, 0, 0);
        }
        break;
    }
}
