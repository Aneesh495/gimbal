/*
 * Gimbal Non-Linear Disturbance Observer (DOB)
 * Estimates unmodeled bearing friction, cable harness spring torque, and wind disturbance.
 */

#include "estimation.h"

void dob_init(disturbance_observer_t *dob, q16_t cutoff_rad_s, const q16_t inertia[3]) {
    if (!dob) return;
    dob->estimated_torque_nm = vec3_zero();
    dob->z_state = vec3_zero();
    dob->cutoff_freq_rad_s = cutoff_rad_s;
    for (int i = 0; i < 3; i++) {
        dob->inertia_kg_m2[i] = inertia ? inertia[i] : q16_from_float(0.005f);
    }
}

vec3_q16_t dob_update(disturbance_observer_t *dob, vec3_q16_t commanded_torque, vec3_q16_t measured_vel, q16_t dt_sec) {
    if (!dob) return vec3_zero();
    
    /*
     * Disturbance observer:
     * tau_dist_est = z + L * J * omega
     * dot(z) = -L * z - L * (tau_cmd + L * J * omega)
     */
    q16_t L = dob->cutoff_freq_rad_s;
    
    vec3_q16_t J_omega = vec3_create(
        q16_mul(dob->inertia_kg_m2[0], measured_vel.x),
        q16_mul(dob->inertia_kg_m2[1], measured_vel.y),
        q16_mul(dob->inertia_kg_m2[2], measured_vel.z)
    );
    
    vec3_q16_t LJ_omega = vec3_scale(J_omega, L);
    
    dob->estimated_torque_nm = vec3_add(dob->z_state, LJ_omega);
    
    vec3_q16_t z_dot = vec3_scale(
        vec3_sub(vec3_zero(), vec3_add(dob->z_state, vec3_add(commanded_torque, LJ_omega))),
        L
    );
    
    dob->z_state = vec3_add(dob->z_state, vec3_scale(z_dot, dt_sec));
    return dob->estimated_torque_nm;
}
