// vfo.c — Numerically Controlled Oscillator (NCO) for sBitx
//
// Originally written by Farhan (VU2ESE) as a real-only sine oscillator.
// Extended by Mike Johnshoy to provide quadrature (I/Q) output for complex
// mixing
//
// vfo_read_iq() returns both the sine (Q) and cosine (I) of the current phase.
// Cosine is derived from the identity:  cos(θ) = sin(θ + 90°)
// In 16-bit phase counts, 90° = 16384 counts.
// Both values are computed in a single quadrant dispatch — no second lookup
// is needed — so the cost of quadrature output is essentially the same as
// real-only output.

#include "sdr.h"
#include <complex.h>
#include <fftw3.h>
#include <linux/types.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

// First-quadrant sine lookup table (0° to 90°, inclusive of both endpoints).
// This table is filled once at startup and never written again.
static int phase_table[MAX_PHASE_COUNT];

// Used to convert a frequency in Hz to a phase increment per sample.
int sampling_freq = 96000;

// Fills the first-quadrant sine table using floating-point math.
// Called once at startup. After this, all oscillator output is computed
// entirely in integer arithmetic using quadrant symmetry.
void vfo_init_phase_table() {
  for (int i = 0; i < MAX_PHASE_COUNT; i++) {
    double d = (M_PI / 2) * ((double)i) / ((double)MAX_PHASE_COUNT);
    phase_table[i] = (int)(sin(d) * 1073741824.0);
  }
}

// Initialises a VFO for a given frequency and starting phase.
void vfo_start(struct vfo *v, int frequency_hz, int start_phase) {
  v->phase_increment = (frequency_hz * 65536) / sampling_freq;
  v->phase = start_phase;
  v->freq_hz = frequency_hz;
}

// Returns the sine of the current phase as a fixed-point integer scaled
// by 2^30, then advances the phase accumulator by one sample.
// For callers that only need a real (single-channel) output. Callers that
// need both I and Q should use vfo_read_iq() instead.
int vfo_read(struct vfo *v) {
  int phase = v->phase & 0xffff;
  int val;

  if (phase < 16384)
    val = phase_table[phase];
  else if (phase < 32768)
    val = phase_table[32767 - phase];
  else if (phase < 49152)
    val = -phase_table[phase - 32768];
  else
    val = -phase_table[65535 - phase];

  v->phase += v->phase_increment;
  v->phase &= 0xffff;
  return val;
}

// Returns both the sine (Q) and cosine (I) of the current phase as
// fixed-point integers scaled by 2^30, then advances the phase accumulator.
// Both channels are resolved in a single quadrant dispatch. Because I and Q
// are always exactly one quadrant apart, one branch determines the correct
// table index for both channels simultaneously
void vfo_read_iq(struct vfo *v, int *out_i, int *out_q) {
  int phase = v->phase & 0xffff;
  int idx;

  if (phase < 16384) {
    idx = phase;
    *out_q =  phase_table[idx];
    *out_i =  phase_table[16383 - idx];
  } else if (phase < 32768) {
    idx = phase - 16384;
    *out_q =  phase_table[16383 - idx];
    *out_i = -phase_table[idx];
  } else if (phase < 49152) {
    idx = phase - 32768;
    *out_q = -phase_table[idx];
    *out_i = -phase_table[16383 - idx];
  } else {
    idx = phase - 49152;
    *out_q = -phase_table[16383 - idx];
    *out_i =  phase_table[idx];
  }

  v->phase += v->phase_increment;
  v->phase &= 0xffff;
}
