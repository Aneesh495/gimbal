/*
 * Gimbal Forward and Inverse Kinematics
 * Maps between actuator joint angles (Pan, Tilt, Roll) and Line-of-Sight pointing quaternions.
 */

#include "kinematics.h"

mat3_q16_t gimbal_forward_kinematics(vec3_q16_t joint_angles) {
    /* Successive rotations: R_gimbal = R_z(pan) * R_y(tilt) * R_x(roll) */
    return mat3_from_euler_zyx(joint_angles.x, joint_angles.y, joint_angles.z);
}

bool gimbal_inverse_kinematics(quat_q16_t desired_pointing, vec3_q16_t *joint_angles_out) {
    if (!joint_angles_out) return false;
    
    q16_t yaw, pitch, roll;
    quat_to_euler_zyx(desired_pointing, &yaw, &pitch, &roll);
    
    /* Check elevation mechanical limits (+/- 85 degrees) to avoid gimbal lock */
    q16_t limit_85deg = 97236; /* 85 * pi/180 in Q16 */
    if (q16_abs(pitch) > limit_85deg) {
        pitch = (pitch > 0) ? limit_85deg : -limit_85deg;
    }
    
    joint_angles_out->x = yaw;
    joint_angles_out->y = pitch;
    joint_angles_out->z = roll;
    return true;
}
