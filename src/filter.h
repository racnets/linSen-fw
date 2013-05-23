#ifndef FILTER_H_
#define FILTER_H_

/* digital biquad filter structure
 *
 * H(z)=fact * \frac{b_0+b_1z^{-1}+b_2z^{-2}} {1+a_1z^{-1}+a_2z^{-2} }
 *
 */
typedef struct {
    int fact;
    int b_0;
    int b_1;
    int b_2;
    int a_1;
    int a_2;
} biquad_t;

typedef struct {
    int x_1;
    int x_2;
    int y_1;
    int y_2;
} filter_t;

extern int update_filter(int x, filter_t* filter, const biquad_t coeffs, const int fpam_exp);
extern int correlation(const int dfo, const int hfo, const int fpam_exp);

#endif /* FILTER_H_ */
