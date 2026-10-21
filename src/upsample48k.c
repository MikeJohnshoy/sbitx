// upsample48k.c
//
// The same prototype lowpass as decim48k.c: for a factor of 2, the
// anti-imaging filter of an interpolator and the anti-aliasing filter of a
// decimator are the same filter.

#include "upsample48k.h"

static const double fir_coeffs[UPSAMPLE48K_TAPS] = {
     0.00359949,  0.00298201, -0.00687610, -0.01768023, -0.00898327,
     0.02024500,  0.03267755, -0.00730499, -0.06718464, -0.05092443,
     0.09648483,  0.29725521,
     0.39168302,
     0.29725521,  0.09648483, -0.05092443, -0.06718464, -0.00730499,
     0.03267755,  0.02024500, -0.00898327, -0.01768023, -0.00687610,
     0.00298201,  0.00359949,
};

// Every other sample going into the filter is an inserted zero, which
// halves the level; this restores it.
#define UPSAMPLE48K_GAIN 2.0

static double push_and_convolve(struct upsample48k_state *f, double x) {
    f->hist[f->pos] = x;
    f->hist[f->pos + UPSAMPLE48K_TAPS] = x;

    int base = f->pos + 1;
    f->pos++;
    if (f->pos == UPSAMPLE48K_TAPS) f->pos = 0;

    double acc = 0.0;
    for (int i = 0; i < UPSAMPLE48K_TAPS; i++)
        acc += fir_coeffs[i] * f->hist[base + i];
    return acc;
}

void upsample48k_apply(struct upsample48k_state *f, double x, double out[2]) {
    out[0] = UPSAMPLE48K_GAIN * push_and_convolve(f, x);
    out[1] = UPSAMPLE48K_GAIN * push_and_convolve(f, 0.0);
}
