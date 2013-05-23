#ifndef __QUADPIXEMD_H
#define __QUADPIXEMD_H

#include "filter.h"

/* datasets */
typedef struct {
    uint8_t update;
    int32_t value[4];
} quad_pix_result_t;

typedef struct {
    uint8_t update;
    int32_t value[10];
} quad_pix_debug_t;

extern quad_pix_result_t quad_pix_result;
extern quad_pix_result_t quad_pix_filter_result;
extern quad_pix_debug_t quad_pix_filter_debug;
/* flags */

/* functions */
void quad_pix_emd_init(int _update_rate);
void quad_pix_emd_update(void);

/* fixed-point arithmetic multiplier(fpam)
 * in filter.c the multiplication is done in 32bitx32bit = 64bit (all signed)
 * the accumulation is done in 64bit + 64bit = 64bit (signed)
 * so only input, factor and the result must fit in 32bit(signed)
 * max. gain ~ 8 per stage(accumulate absolute coefficients)
 * max. input: 2^16 = 2^12 * 2^3 * 2^1(signed)
 * max. factor: 2^3 = 2^2 *2^1(signed)
 * max. result: 2^22 = 5 * 2^3 * 2^16
 * so fpam <= 2^10
 */
#define fpam_exp     10
#define fpam         (1<<fpam_exp)
/* filter coefficients */
/* factor b0 b1 b2 -a1 -a2 */
/* factor = root(filter gain factor, amount of stages) */

/* 6.order low pass 25Hz -3dB 50Hz -10db */
static const biquad_t input_filter_stage_01_biquad = {0.0055973*fpam, 1*fpam, 2.0066925*fpam, 1.0067075*fpam, 1.8985094*fpam, -0.9221746*fpam};
static const biquad_t input_filter_stage_02_biquad = {0.0055973*fpam, 1*fpam, 1.9999772*fpam, 0.9999921*fpam, 1.7786318*fpam, -0.8008026*fpam};
static const biquad_t input_filter_stage_03_biquad = {0.0055973*fpam, 1*fpam, 1.9933303*fpam, 0.9933451*fpam, 1.7160713*fpam, -0.7374623*fpam};

/* high pass 10Hz -3dB */
static const biquad_t high_pass_filter_stage_01_biquad = {0.9695313*fpam, 1*fpam, -1*fpam, 0*fpam, 0.9390625*fpam, 0*fpam};

/* low pass 1Hz -3dB 5Hz -10dB */
static const biquad_t delay_filter_stage_01_biquad = {0.0031318*fpam, 1*fpam, 1*fpam, 0*fpam, 0.9937365*fpam, 0*fpam};

#endif /* __QUADPIXEMD_H */
