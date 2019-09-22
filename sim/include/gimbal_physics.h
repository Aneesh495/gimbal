#ifndef SIM_GIMBAL_PHYSICS_H
#define SIM_GIMBAL_PHYSICS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double angles[3];        /* Pan, Tilt, Roll (rad) */
    double angular_rates[3]; /* rad/s */
    double torques[3];       /* Motor drive torques (Nm) */
    double dist_torques[3];  /* Aerodynamic and base disturbance torques (Nm) */
    
    /* Physical parameters */
    double inertia[3];       /* kg*m^2 for Pan, Tilt, Roll */
    double friction_viscous[3];
    double friction_coulomb[3];
    double friction_stribeck[3];
    double stribeck_vel[3];
    double cable_stiffness[3];
} gimbal_plant_t;

void gimbal_plant_init(gimbal_plant_t *plant);
void gimbal_plant_rk4_step(gimbal_plant_t *plant, double dt_sec);

#endif
