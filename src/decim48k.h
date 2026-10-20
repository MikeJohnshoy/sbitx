// decim48k.h
//
// Decimates 96 kHz audio or I/Q by 2, to 48 kHz: a 25-tap half-band-style
// FIR, flat to 15 kHz, from maxibitx (its usb_uac_decimation_design.md
// has the design). sbitx uses it for TCI's receive audio (tci.h).

#ifndef DECIM48K_H
#define DECIM48K_H

#define DECIM48K_TAPS 25

// One instance per rail - I and Q each need independent history and
// decimation phase, but share the same coefficient table (decim48k.c).
// Zero-initialize (static/global storage does this automatically)
// before first use. The history is written twice (decim48k.c), so the
// filter reads it without wrapping.
struct decim48k_state {
    double hist[2 * DECIM48K_TAPS];
    int pos;     // write cursor, always in [0, DECIM48K_TAPS)
    int phase;   // toggles every call; only every other call emits output
};

// Feed one 96kHz-rate input sample. Every call updates the filter's
// history (needed for correct output on the calls that do emit), but
// only every other call actually produces a decimated 48kHz output
// sample - that call writes *out and returns 1; the discarded call
// returns 0 and leaves *out untouched. Call once per input sample, per
// rail, in lockstep with the other rail (both need to be fed on every
// input sample so their phases stay aligned with each other).
int decim48k_apply(struct decim48k_state *f, double x, double *out);

#endif /* DECIM48K_H */
