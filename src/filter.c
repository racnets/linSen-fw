#include "filter.h"

inline int update_filter(int x, filter_t* filter, const biquad_t coeffs, const int fpam_exp) {
    register unsigned rl = 0;
    register int rh = 0;

    /* direct form 1 implementation
     * use assembler code on arm processor
     *
     *    result = coeffs.b_0 * x + coeffs.b_1 * filter->x_1 + coeffs.b_2 * filter->x_2 \
     *            - coeffs.a_1 * filter->y_1 - coeffs.a_2 * filter->y_2;
     */

    /* result += coeffs.b_0 * x */
    __asm (
        "smlal %[rl], %[rh], %[xn], %[bn];"
            : [rl]    "+l"    (rl),
              [rh]    "+l"    (rh)
            : [xn]    "l"        (x),
              [bn]    "l"        (coeffs.b_0)
    );

    /* result += coeffs.b_1 * x_1 */
    __asm (
        "smlal %[rl], %[rh], %[xn], %[bn];"
            : [rl]    "+l"    (rl),
              [rh]    "+l"    (rh)
            : [xn]    "l"        (filter->x_1),
              [bn]    "l"        (coeffs.b_1)
    );

    /* result += coeffs.b_2 * x_2 */
    __asm (
        "smlal %[rl], %[rh], %[xn], %[bn];"
            : [rl]    "+l"    (rl),
              [rh]    "+l"    (rh)
            : [xn]    "l"        (filter->x_2),
              [bn]    "l"        (coeffs.b_2)
    );

    /* result += coeffs.a_1 * y_1 */
    __asm (
        "smlal %[rl], %[rh], %[yn], %[an];"
            : [rl]    "+l"    (rl),
              [rh]    "+l"    (rh)
            : [yn]    "l"        (filter->y_1),
              [an]    "l"        (coeffs.a_1)
    );

    /* result += coeffs.a_1 * y_1 */
    __asm (
        "smlal %[rl], %[rh], %[yn], %[an];"
            : [rl]    "+l"    (rl),
              [rh]    "+l"    (rh)
            : [yn]    "l"        (filter->y_2),
              [an]    "l"        (coeffs.a_2)
    );

    /* scale down to used fixed point arithmetic ([fpam * fpam / fpam = fpam]) */
    rl = rl >> fpam_exp;
    rh = rh << (32-fpam_exp);
    rl = rh | rl;

    /* update delay registers */
    filter->x_2 = filter->x_1;
    filter->x_1 = x;
    filter->y_2 = filter->y_1;
    filter->y_1 = rl;

    /* factor */
    /* result *= coeffs.fact */
    __asm (
        "smull %[rl], %[rh], %[rl], %[f];"
            : [rl]    "+l"    (rl),
              [rh]    "+l"    (rh)
            : [f]    "l"        (coeffs.fact)
    );

    /* scale down to used fixed point arithmetic ([fpam * fpam / fpam = fpam]) */
    rl = rl >> fpam_exp;
    rh = rh << (32-fpam_exp);
    rl = rh | rl;

    return rl;
}

inline int correlation(const int dfo, const int hfo, const int fpam_exp){
    register unsigned rl = 0;
    register int rh = 0;

    /* c = dfo * hfo; */
    __asm (
        "smull %[rl], %[rh], %[d], %[h];"
            : [rl]    "+l"    (rl),
              [rh]    "+l"    (rh)
            : [d]    "l"        (dfo),
              [h]    "l"        (hfo)
    );
    /* scale down to used fixed point arithmetic ([fpam * fpam / fpam = fpam]) */
    rl = rl >> fpam_exp;
    rh = rh << (32-fpam_exp);
    return (int)(rh | rl);
}
