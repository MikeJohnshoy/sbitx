// tci_sbitx.h
//
// sbitx's side of the TCI server (tci_sbitx.c): starting and stopping it
// from the TCI and TCI_PORT settings, the radio's state published for it,
// and its receive streams. docs/tci_design_study.md.

#ifndef TCI_SBITX_H
#define TCI_SBITX_H

#include <stdint.h>

// The GTK thread, on every tick of ui_tick(): follows the TCI setting,
// starting or stopping the server, and publishes the radio's state for it.
void tci_sbitx_tick(void);

// The audio thread, once per block of n samples at 96 kHz: the
// demodulated audio (sound_process()'s output_speaker) and the I/Q, or
// NULL for both while transmitting, which sends silence.
void tci_sbitx_rx_block(const int32_t *speaker, const double *i, const double *q, int n);

// The GTK thread: nonzero while TCI has turned on VFO LOCK because a
// client has set the frequency (tci_sbitx.c, THE DIAL).
int tci_sbitx_has_dial(void);

#endif /* TCI_SBITX_H */
