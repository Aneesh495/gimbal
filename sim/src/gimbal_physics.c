/*
 * Gimbal Non-Linear 3-DOF Rigid Body Dynamics and 4th-Order Runge-Kutta Integrator
 * Simulates Euler equations of rotational motion, Stribeck friction curves, and harness stiffness.
 */

#include "gimbal_physics.h"
#include <math.h>

void gimbal_plant_init(gimbal_plant_t *plant) {
    if (!plant) return;
    
    for (int i = 0; i < 3; i++) {
        plant->angles[i] = 0.0;
        plant->angular_rates[i] = 0.0;
        plant->torques[i] = 0.0;
        plant->dist_torques[i] = 0.0;
        
        plant->friction_viscous[i] = 0.002;
        plant->friction_coulomb[i] = 0.015;
        plant->friction_stribeck[i] = 0.035;
        plant->stribeck_vel[i] = 0.05;
        plant->cable_stiffness[i] = 0.010;
    }
    
    /* Pan, Tilt, Roll inertia */
    plant->inertia[0] = 0.015; /* Pan (outer yoke) */
    plant->inertia[1] = 0.009; /* Tilt (inner arm) */
    plant->inertia[2] = 0.005; /* Roll (optical payload) */
}

static void evaluate_derivatives(const gimbal_plant_t *plant, const double x[6], double dxdt[6]) {
    for (int i = 0; i < 3; i++) {
        double angle = x[i];
        double rate  = x[i + 3];
        
        /* 1. Stribeck non-linear friction model:
         * F_fric = [F_c + (F_s - F_c) * exp(-(v/v_s)^2)] * sgn(v) + F_v * v
         */
        double v_ratio = rate / (plant->stribeck_vel[i] + 1e-9);
        double s_decay = exp(-(v_ratio * v_ratio));
        double f_dry = plant->friction_coulomb[i] + (plant->friction_stribeck[i] - plant->friction_coulomb[i]) * s_decay;
        double sign_v = (rate > 1e-4) ? 1.0 : ((rate < -1e-4) ? -1.0 : rate * 10000.0);
        double tau_friction = (f_dry * sign_v) + (plant->friction_viscous[i] * rate);
        
        /* 2. Cable harness spring torque */
        double tau_cable = plant->cable_stiffness[i] * angle;
        
        /* 3. Net torque = Motor + Disturbance - Friction - Cable */
        double tau_net = plant->torques[i] + plant->dist_torques[i] - tau_friction - tau_cable;
        
        /* Angular acceleration: alpha = tau_net / J */
        double alpha = tau_net / plant->inertia[i];
        
        dxdt[i] = rate;
        dxdt[i + 3] = alpha;
    }
}

void gimbal_plant_rk4_step(gimbal_plant_t *plant, double dt_sec) {
    if (!plant || dt_sec <= 0.0) return;
    
    double x[6], k1[6], k2[6], k3[6], k4[6], x_temp[6];
    for (int i = 0; i < 3; i++) {
        x[i] = plant->angles[i];
        x[i + 3] = plant->angular_rates[i];
    }
    
    /* k1 */
    evaluate_derivatives(plant, x, k1);
    
    /* k2 */
    for (int i = 0; i < 6; i++) x_temp[i] = x[i] + 0.5 * dt_sec * k1[i];
    evaluate_derivatives(plant, x_temp, k2);
    
    /* k3 */
    for (int i = 0; i < 6; i++) x_temp[i] = x[i] + 0.5 * dt_sec * k2[i];
    evaluate_derivatives(plant, x_temp, k3);
    
    /* k4 */
    for (int i = 0; i < 6; i++) x_temp[i] = x[i] + dt_sec * k3[i];
    evaluate_derivatives(plant, x_temp, k4);
    
    /* Integration update */
    for (int i = 0; i < 3; i++) {
        plant->angles[i] += (dt_sec / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        plant->angular_rates[i] += (dt_sec / 6.0) * (k1[i+3] + 2.0*k2[i+3] + 2.0*k3[i+3] + k4[i+3]);
    }
}
