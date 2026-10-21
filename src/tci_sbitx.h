// tci_sbitx.h
//
// sbitx's side of the TCI server (tci_sbitx.c): starting and stopping it
// from the TCI and TCI_PORT settings, the radio's state published for it,
// its receive streams and transmit audio. docs/tci_design_study.md.

#ifndef TCI_SBITX_H
#define TCI_SBITX_H

#include <stdint.h>

// The GTK thread, on every tick of ui_tick(): follows the TCI setting,
// starting or stopping the server, and publishes the radio's state for it.
void tci_sbitx_tick(void);

// The audio thread, once per block of n samples at 96 kHz: the
// demodulated audio (sound_process()'s output_speaker) and the I/Q, or
// NULL for both while transmitting, which sends silence. The I/Q is
// shifted up by shift_hz (sbitx.c: to centre it on tci_iq_center(), and
// in CW by the pitch); a shift beyond its width sends silence.
void tci_sbitx_rx_block(const int32_t *speaker, const double *i, const double *q, int n,
                        int shift_hz);

// The audio thread, once per block while transmitting, before
// tx_process(): if a client holds TX with its own audio and the mode takes
// it (USB, LSB, DIGI, FT8, FT4), fills mic[0..n-1] (96 kHz, int32) with it,
// or with silence for the last blocks after the client unkeyed, and
// returns 1; otherwise returns 0 and leaves mic alone.
int tci_sbitx_tx_block(int32_t *mic, int n, int mode);

// The audio thread, once per block while receiving: discards any transmit
// audio a client sent late, unless a client has just keyed and sbitx
// hasn't started transmitting yet.
void tci_sbitx_tx_idle(void);

// Any thread: nonzero while a client holds TX, or its TX has been unkeyed
// and sbitx hasn't finished ending it. sbitx's own FT8/FT4 modem stands
// aside meanwhile (modems.c, tx_process()).
int tci_sbitx_tx_engaged(void);

#endif /* TCI_SBITX_H */
