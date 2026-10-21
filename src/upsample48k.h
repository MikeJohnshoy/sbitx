// upsample48k.h
//
// Upsamples 48 kHz audio by 2, to 96 kHz: zero-stuffing then decim48k.c's
// 25-tap lowpass (flat to 15 kHz, 54 dB down from 24 kHz) with a gain of 2,
// from maxibitx. sbitx uses it for a TCI program's transmit audio
// (tci_sbitx.c).

#ifndef UPSAMPLE48K_H
#define UPSAMPLE48K_H

#define UPSAMPLE48K_TAPS 25

// Zero-initialize (static storage does) before first use. The history is
// written twice (upsample48k.c), so the filter reads it without wrapping.
struct upsample48k_state {
    double hist[2 * UPSAMPLE48K_TAPS];
    int pos; // write cursor, always in [0, UPSAMPLE48K_TAPS)
};

// One 48 kHz sample in, the two 96 kHz samples it becomes out, in order.
void upsample48k_apply(struct upsample48k_state *f, double x, double out[2]);

#endif /* UPSAMPLE48K_H */
