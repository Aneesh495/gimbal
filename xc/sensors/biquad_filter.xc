/*
 * Gimbal Direct Form II Transposed Biquad IIR Filter
 * 64-bit fixed-point Q31 precision for high-order lowpass, bandpass, and notch filtering.
 */

#include "sensors.h"
#include "fixed_math.h"

void biquad_init_notch(biquad_df2t_t *filter, q16_t f0_hz, q16_t bw_hz, q16_t fs_hz) {
    if (!filter || fs_hz <= 0) return;
    
    float f0 = q16_to_float(f0_hz);
    float bw = q16_to_float(bw_hz);
    float fs = q16_to_float(fs_hz);
    
    float w0 = 2.0f * 3.14159265f * f0 / fs;
    float sin_w0 = 0.0f;
    float cos_w0 = 1.0f;
    
    q16_t w0_q16 = q16_from_float(w0);
    q16_t s_q, c_q;
    q16_cordic_sin_cos(w0_q16, &s_q, &c_q);
    sin_w0 = q16_to_float(s_q);
    cos_w0 = q16_to_float(c_q);
    
    float alpha = sin_w0 * (bw / (2.0f * f0));
    float b0 = 1.0f;
    float b1 = -2.0f * cos_w0;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;
    
    /* Normalize by a0 and convert to Q31 */
    filter->b0 = (q31_t)((b0 / a0) * 2147483648.0f);
    filter->b1 = (q31_t)((b1 / a0) * 2147483648.0f);
    filter->b2 = (q31_t)((b2 / a0) * 2147483648.0f);
    filter->a1 = (q31_t)((a1 / a0) * 2147483648.0f);
    filter->a2 = (q31_t)((a2 / a0) * 2147483648.0f);
    
    filter->s1 = 0;
    filter->s2 = 0;
}

void biquad_init_lowpass(biquad_df2t_t *filter, q16_t fc_hz, q16_t q_factor, q16_t fs_hz) {
    if (!filter || fs_hz <= 0) return;
    
    float fc = q16_to_float(fc_hz);
    float q = q16_to_float(q_factor);
    float fs = q16_to_float(fs_hz);
    
    float w0 = 2.0f * 3.14159265f * fc / fs;
    q16_t w0_q16 = q16_from_float(w0);
    q16_t s_q, c_q;
    q16_cordic_sin_cos(w0_q16, &s_q, &c_q);
    float sin_w0 = q16_to_float(s_q);
    float cos_w0 = q16_to_float(c_q);
    
    float alpha = sin_w0 / (2.0f * q);
    float b0 = (1.0f - cos_w0) * 0.5f;
    float b1 = 1.0f - cos_w0;
    float b2 = (1.0f - cos_w0) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;
    
    filter->b0 = (q31_t)((b0 / a0) * 2147483648.0f);
    filter->b1 = (q31_t)((b1 / a0) * 2147483648.0f);
    filter->b2 = (q31_t)((b2 / a0) * 2147483648.0f);
    filter->a1 = (q31_t)((a1 / a0) * 2147483648.0f);
    filter->a2 = (q31_t)((a2 / a0) * 2147483648.0f);
    
    filter->s1 = 0;
    filter->s2 = 0;
}

q16_t biquad_process_sample(biquad_df2t_t *filter, q16_t input) {
    /* Convert Q16 input to Q31 */
    q31_t in_q31 = input << 15;
    
    /* Direct Form II Transposed:
     * y[n] = b0*x[n] + s1[n-1]
     * s1[n] = b1*x[n] - a1*y[n] + s2[n-1]
     * s2[n] = b2*x[n] - a2*y[n]
     */
    int64_t b0_x = ((int64_t)filter->b0 * in_q31) >> 31;
    q31_t y_q31 = (q31_t)(b0_x + filter->s1);
    
    int64_t b1_x = ((int64_t)filter->b1 * in_q31) >> 31;
    int64_t a1_y = ((int64_t)filter->a1 * y_q31) >> 31;
    filter->s1 = (q31_t)(b1_x - a1_y + filter->s2);
    
    int64_t b2_x = ((int64_t)filter->b2 * in_q31) >> 31;
    int64_t a2_y = ((int64_t)filter->a2 * y_q31) >> 31;
    filter->s2 = (q31_t)(b2_x - a2_y);
    
    return (q16_t)(y_q31 >> 15);
}

void biquad_reset(biquad_df2t_t *filter) {
    if (filter) {
        filter->s1 = 0;
        filter->s2 = 0;
    }
}
