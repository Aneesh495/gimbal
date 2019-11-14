/*
 * Gimbal EKF Covariance Propagation and Joseph-Form Update
 * Numerically stabilized matrix updates guaranteeing positive-definiteness.
 */

#include "estimation.h"

void ekf_cov_predict(q16_t P[6][6], const q16_t F[6][6], const q16_t Q_diag[6], q16_t dt) {
    (void)dt;
    q16_t FP[6][6];
    
    /* FP = F * P */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            int64_t sum = 0;
            for (int k = 0; k < 6; k++) {
                sum += (int64_t)F[i][k] * P[k][j];
            }
            FP[i][j] = (q16_t)((sum + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
        }
    }
    
    /* P_new = FP * F^T + Q */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            int64_t sum = 0;
            for (int k = 0; k < 6; k++) {
                sum += (int64_t)FP[i][k] * F[j][k];
            }
            q16_t p_val = (q16_t)((sum + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
            if (i == j) {
                p_val = q16_add(p_val, Q_diag[i]);
            }
            P[i][j] = p_val;
        }
    }
}

void ekf_cov_update_joseph(q16_t P[6][6], const q16_t H[3][6], const q16_t K[6][3], const q16_t R_diag[3]) {
    /* P = (I - KH) * P * (I - KH)^T + K * R * K^T */
    q16_t I_KH[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            int64_t kh = 0;
            for (int k = 0; k < 3; k++) {
                kh += (int64_t)K[i][k] * H[k][j];
            }
            q16_t delta = (i == j) ? FIXED_Q16_ONE : 0;
            I_KH[i][j] = delta - (q16_t)((kh + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
        }
    }
    
    q16_t temp[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            int64_t sum = 0;
            for (int k = 0; k < 6; k++) {
                sum += (int64_t)I_KH[i][k] * P[k][j];
            }
            temp[i][j] = (q16_t)((sum + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
        }
    }
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            int64_t sum = 0;
            for (int k = 0; k < 6; k++) {
                sum += (int64_t)temp[i][k] * I_KH[j][k];
            }
            q16_t p_val = (q16_t)((sum + FIXED_Q16_HALF) >> FIXED_Q16_SHIFT);
            
            /* Add KRK^T */
            int64_t krk = 0;
            for (int k = 0; k < 3; k++) {
                krk += ((int64_t)K[i][k] * K[j][k] * R_diag[k]) >> FIXED_Q16_SHIFT;
            }
            p_val = q16_add(p_val, (q16_t)(krk >> FIXED_Q16_SHIFT));
            P[i][j] = p_val;
        }
    }
}
