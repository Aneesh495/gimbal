/*
 * Gimbal Field Weakening and Maximum Torque Per Ampere (MTPA) Governor
 * Injects negative Id current when back-EMF voltage approaches inverter bus limit.
 */

#include "foc.h"

void foc_field_weakening_update(motor_axis_channel_t *axis, q16_t electrical_speed) {
    if (!axis) return;
    
    q16_t speed_mag = q16_abs(electrical_speed);
    q16_t base_speed = q16_from_float(150.0f); /* rad/s electrical base speed */
    
    if (speed_mag > base_speed) {
        /* Voltage deficit proportional to excess speed */
        q16_t excess_speed = speed_mag - base_speed;
        q16_t id_inject = q16_mul(excess_speed, q16_from_float(0.02f));
        
        /* Clamp negative Id current to 30% of max motor current */
        q16_t max_id = q16_mul(axis->params.max_current_amps, q16_from_float(0.30f));
        if (id_inject > max_id) id_inject = max_id;
        
        axis->target_id = -id_inject;
    } else {
        axis->target_id = 0;
    }
}
