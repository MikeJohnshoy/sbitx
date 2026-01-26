// CESSB (Controlled Envelope Single Sideband) Processing Implementation
//
// Reference: "Controlled Envelope Single Sideband" by David Hershberger, W9GR
//            QEX November/December 2014
//
// Concept: CESSB increases average SSB transmit power by ~2–3 dB without increasing peak
// envelope power (PEP). This is achieved by controlling envelope overshoot that
// normally occurs when clipped audio is filtered.
//
// Processing chain:
// - Input audio samples (integers), converted to float (±0.04)
// - Normalize to ±1.0
// - Hilbert transform (127 taps)
// - Hard clip (envelope-based) to CESSB_CLIP_LEVEL
// - Overshoot-control Blackman–Harris-windowed LPF @ 3 kHz prevents overshoot regeneration
// - Hilbert transform to re-measure envelope after filtering
// - Look-ahead final envelope limiting with configurable look-ahead (up to 1024 samples)
//   and smooth attack/release gain control
// - Post-limiter LPF: 6th-order Butterworth @ 3 kHz
//   Removes residual out-of-band content
// - Scale to ±0.04 and convert back to integer before
//   returning processed data to tx_process pipeline
//
// Key configuration parameters (see cessb.h)
// There are also functions provided to set these if a control panel is needed.
//
//   CESSB_PRE_GAIN                Add gain so float values reach +/- 1
//   CESSB_CLIP_LEVEL              Initial clip threshold (default 0.85)
//   CESSB_ENVELOPE_LIMIT          Final limiter ceiling (default 1.0)
//   LOOKAHEAD_DEFAULT_SAMPLES     Default look-ahead (~2 ms at 96 kHz)
//   LOOKAHEAD_DEFAULT_ATTACK_MS   Limiter attack (default 0.5 ms)
//   LOOKAHEAD_DEFAULT_RELEASE_MS  Limiter release (default 50 ms)
//
// Added by Mike KB2ML and Bob KD8CGH

#include <math.h>
#include <stdio.h>  // for debug print statements only
#include <stdlib.h>
#include <string.h>

#include "cessb.h"

// ============================================================================
// PRECOMPUTED FILTER COEFFICIENTS
// Generated for: 96000 Hz sample rate, 3000 Hz audio cutoff
// ============================================================================

// Hilbert transform filter (127 taps, Blackman window)
static const float hilbert_coeffs[HILBERT_TAPS] = {
     1.40236097e-19f,  0.00000000e+00f, -9.37617092e-06f,  0.00000000e+00f, -3.91882957e-05f,
     0.00000000e+00f, -9.28426066e-05f,  0.00000000e+00f, -1.75022280e-04f,  0.00000000e+00f,
    -2.91799050e-04f,  0.00000000e+00f, -4.50725486e-04f,  0.00000000e+00f, -6.60910489e-04f,
     0.00000000e+00f, -9.33083247e-04f,  0.00000000e+00f, -1.27965419e-03f,  0.00000000e+00f,
    -1.71478569e-03f,  0.00000000e+00f, -2.25449075e-03f,  0.00000000e+00f, -2.91678511e-03f,
     0.00000000e+00f, -3.72192860e-03f,  0.00000000e+00f, -4.69280630e-03f,  0.00000000e+00f,
    -5.85552231e-03f,  0.00000000e+00f, -7.24031348e-03f,  0.00000000e+00f, -8.88294600e-03f,
     0.00000000e+00f, -1.08268500e-02f,  0.00000000e+00f, -1.31264051e-02f,  0.00000000e+00f,
    -1.58520727e-02f,  0.00000000e+00f, -1.90985932e-02f,  0.00000000e+00f, -2.29984892e-02f,
     0.00000000e+00f, -2.77452162e-02f,  0.00000000e+00f, -3.36349290e-02f,  0.00000000e+00f,
    -4.11468657e-02f,  0.00000000e+00f, -5.11114405e-02f,  0.00000000e+00f, -6.51024086e-02f,
     0.00000000e+00f, -8.65011541e-02f,  0.00000000e+00f, -1.24114929e-01f,  0.00000000e+00f,
    -2.10267285e-01f,  0.00000000e+00f, -6.35971008e-01f,  0.00000000e+00f,  6.35971008e-01f,
     0.00000000e+00f,  2.10267285e-01f,  0.00000000e+00f,  1.24114929e-01f,  0.00000000e+00f,
     8.65011541e-02f,  0.00000000e+00f,  6.51024086e-02f,  0.00000000e+00f,  5.11114405e-02f,
     0.00000000e+00f,  4.11468657e-02f,  0.00000000e+00f,  3.36349290e-02f,  0.00000000e+00f,
     2.77452162e-02f,  0.00000000e+00f,  2.29984892e-02f,  0.00000000e+00f,  1.90985932e-02f,
     0.00000000e+00f,  1.58520727e-02f,  0.00000000e+00f,  1.31264051e-02f,  0.00000000e+00f,
     1.08268500e-02f,  0.00000000e+00f,  8.88294600e-03f,  0.00000000e+00f,  7.24031348e-03f,
     0.00000000e+00f,  5.85552231e-03f,  0.00000000e+00f,  4.69280630e-03f,  0.00000000e+00f,
     3.72192860e-03f,  0.00000000e+00f,  2.91678511e-03f,  0.00000000e+00f,  2.25449075e-03f,
     0.00000000e+00f,  1.71478569e-03f,  0.00000000e+00f,  1.27965419e-03f,  0.00000000e+00f,
     9.33083247e-04f,  0.00000000e+00f,  6.60910489e-04f,  0.00000000e+00f,  4.50725486e-04f,
     0.00000000e+00f,  2.91799050e-04f,  0.00000000e+00f,  1.75022280e-04f,  0.00000000e+00f,
     9.28426066e-05f,  0.00000000e+00f,  3.91882957e-05f,  0.00000000e+00f,  9.37617092e-06f,
     0.00000000e+00f, -1.40236097e-19f
};

// Overshoot control filter (65 taps, Blackman-Harris window)
// Lowpass, cutoff 3000 Hz, normalized fc = 0.031250
static const float overshoot_coeffs[OVERSHOOT_FILTER_TAPS] = {
    -1.55789393e-22f, -4.25971371e-07f, -2.84081374e-06f, -1.00466862e-05f, -2.62075167e-05f,
    -5.70680120e-05f, -1.09643317e-04f, -1.91176866e-04f, -3.07228071e-04f, -4.58862990e-04f,
    -6.39078438e-04f, -8.28780584e-04f, -9.92837463e-04f, -1.07689533e-03f, -1.00574960e-03f,
    -6.84053239e-04f,  5.64658657e-19f,  1.16767022e-03f,  2.93941582e-03f,  5.42196095e-03f,
     8.69377583e-03f,  1.27905065e-02f,  1.76921839e-02f,  2.33141094e-02f,  2.95031125e-02f,
     3.60404042e-02f,  4.26515434e-02f,  4.90231806e-02f,  5.48253530e-02f,  5.97373084e-02f,
     6.34742558e-02f,  6.58121708e-02f,  6.66078871e-02f,  6.58121708e-02f,  6.34742558e-02f,
     5.97373084e-02f,  5.48253530e-02f,  4.90231806e-02f,  4.26515434e-02f,  3.60404042e-02f,
     2.95031125e-02f,  2.33141094e-02f,  1.76921839e-02f,  1.27905065e-02f,  8.69377583e-03f,
     5.42196095e-03f,  2.93941582e-03f,  1.16767022e-03f,  5.64658657e-19f, -6.84053239e-04f,
    -1.00574960e-03f, -1.07689533e-03f, -9.92837463e-04f, -8.28780584e-04f, -6.39078438e-04f,
    -4.58862990e-04f, -3.07228071e-04f, -1.91176866e-04f, -1.09643317e-04f, -5.70680120e-05f,
    -2.62075167e-05f, -1.00466862e-05f, -2.84081374e-06f, -4.25971371e-07f, -1.55789393e-22f
};

// Post-limiter LPF (6th-order Butterworth, 3000 Hz)
// Each row: b0, b1, b2, a1, a2
static const float post_lpf_coeffs[POST_LPF_BIQUAD_STAGES][5] = {
    {  8.08399021e-03f,  1.61679804e-02f,  8.08399021e-03f, -1.65053850e+00f,  6.82874458e-01f },
    {  8.44269293e-03f,  1.68853859e-02f,  8.44269293e-03f, -1.72377617e+00f,  7.57546944e-01f },
    {  9.14557162e-03f,  1.82911432e-02f,  9.14557162e-03f, -1.86728554e+00f,  9.03867829e-01f }
};

// Global instance
int cessb_enabled = CESSB_DISABLED;
cessb_state_t cessb_processor;

// ============================================================================
// ATTACK/RELEASE COEFFICIENT CALCULATION
// ============================================================================

static float time_constant_to_coeff(float time_ms, float sample_rate) {
  if (time_ms <= 0.0f) return 1.0f;
  float time_samples = (time_ms / 1000.0f) * sample_rate;
  return 1.0f - expf(-1.0f / time_samples);
}

// ============================================================================
// FIR FILTER PROCESSING
// ============================================================================

static float apply_fir_filter(const float *coeffs, float *delay, int *index, int num_taps,
                              float input) {
  delay[*index] = input;

  float output = 0.0f;
  int idx = *index;

  for (int i = 0; i < num_taps; i++) {
    output += coeffs[i] * delay[idx];
    idx--;
    if (idx < 0) idx = num_taps - 1;
  }

  (*index)++;
  if (*index >= num_taps) *index = 0;

  return output;
}

static float get_delayed_sample(float *delay, int *index, int delay_length, float input) {
  float output = delay[*index];
  delay[*index] = input;

  (*index)++;
  if (*index >= delay_length) *index = 0;

  return output;
}

// ============================================================================
// BIQUAD FILTER PROCESSING
// ============================================================================

static float apply_biquad(const float *coeffs, biquad_state_t *state, float input) {
  float output = coeffs[0] * input + coeffs[1] * state->x1 + coeffs[2] * state->x2 -
                 coeffs[3] * state->y1 - coeffs[4] * state->y2;

  state->x2 = state->x1;
  state->x1 = input;
  state->y2 = state->y1;
  state->y1 = output;

  return output;
}

static float apply_biquad_cascade(const float coeffs[][5], biquad_state_t *states,
                                  int num_stages, float input) {
  float output = input;
  for (int i = 0; i < num_stages; i++) {
    output = apply_biquad(coeffs[i], &states[i], output);
  }
  return output;
}

// ============================================================================
// LOOK-AHEAD LIMITER
// ============================================================================

static void lookahead_limiter_init(lookahead_limiter_t *lim, float sample_rate) {
  memset(lim->delay, 0, sizeof(lim->delay));
  memset(lim->envelope, 0, sizeof(lim->envelope));
  lim->write_index = 0;
  lim->lookahead_samples = LOOKAHEAD_DEFAULT_SAMPLES;
  lim->current_gain = 1.0f;
  lim->peak_hold = 0.0f;

  lim->attack_coeff = time_constant_to_coeff(LOOKAHEAD_DEFAULT_ATTACK_MS, sample_rate);
  lim->release_coeff = time_constant_to_coeff(LOOKAHEAD_DEFAULT_RELEASE_MS, sample_rate);
}

static float find_peak_in_window(lookahead_limiter_t *lim) {
  float peak = 0.0f;
  int read_index = lim->write_index;

  for (int i = 0; i < lim->lookahead_samples; i++) {
    if (lim->envelope[read_index] > peak) {
      peak = lim->envelope[read_index];
    }
    read_index++;
    if (read_index >= LOOKAHEAD_MAX_SAMPLES) read_index = 0;
  }

  return peak;
}

static float lookahead_limiter_process(lookahead_limiter_t *lim, float input,
                                       float envelope, float limit) {
  lim->delay[lim->write_index] = input;
  lim->envelope[lim->write_index] = envelope;

  int read_index = lim->write_index - lim->lookahead_samples;
  if (read_index < 0) read_index += LOOKAHEAD_MAX_SAMPLES;

  float peak_envelope = find_peak_in_window(lim);

  float target_gain = 1.0f;
  if (peak_envelope > limit && peak_envelope > 1e-10f) {
    target_gain = limit / peak_envelope;
  }

  if (target_gain < lim->current_gain) {
    lim->current_gain += lim->attack_coeff * (target_gain - lim->current_gain);
  } else {
    lim->current_gain += lim->release_coeff * (target_gain - lim->current_gain);
  }

  if (lim->current_gain < 0.0f) lim->current_gain = 0.0f;
  if (lim->current_gain > 1.0f) lim->current_gain = 1.0f;

  float output = lim->delay[read_index] * lim->current_gain;

  lim->write_index++;
  if (lim->write_index >= LOOKAHEAD_MAX_SAMPLES) lim->write_index = 0;

  return output;
}

// ============================================================================
// CESSB INITIALIZATION AND CONFIGURATION
// ============================================================================

void cessb_init(cessb_state_t *state, float sample_rate) {
  memset(state, 0, sizeof(cessb_state_t));

  state->enabled = CESSB_DISABLED;
  state->clip_level = CESSB_CLIP_LEVEL;
  state->envelope_limit = CESSB_ENVELOPE_LIMIT;
  state->sample_rate = sample_rate;

  state->hilbert_index = 0;
  state->delay_index = 0;
  state->overshoot_index = 0;
  state->hilbert2_index = 0;
  state->delay2_index = 0;

  lookahead_limiter_init(&state->lookahead, sample_rate);
  cessb_reset_stats(state);
}

void cessb_set_enabled(cessb_state_t *state, int enabled) {
  state->enabled = enabled;
  cessb_enabled = enabled;
}

void cessb_set_clip_level(cessb_state_t *state, float level) {
  if (level > 0.0f && level <= 1.0f) {
    state->clip_level = level;
  }
}

void cessb_set_envelope_limit(cessb_state_t *state, float limit) {
  if (limit > 0.0f && limit <= 1.5f) {
    state->envelope_limit = limit;
  }
}

int cessb_is_enabled(cessb_state_t *state) { 
     return state->enabled; 
}

// ============================================================================
// LOOK-AHEAD LIMITER CONFIGURATION
// ============================================================================

void cessb_set_lookahead_samples(cessb_state_t *state, int samples) {
  if (samples < 1) samples = 1;
  if (samples > LOOKAHEAD_MAX_SAMPLES) samples = LOOKAHEAD_MAX_SAMPLES;
  state->lookahead.lookahead_samples = samples;
}

void cessb_set_lookahead_ms(cessb_state_t *state, float milliseconds) {
  int samples = (int)((milliseconds / 1000.0f) * state->sample_rate + 0.5f);
  cessb_set_lookahead_samples(state, samples);
}

void cessb_set_attack_ms(cessb_state_t *state, float attack_ms) {
  state->lookahead.attack_coeff = time_constant_to_coeff(attack_ms, state->sample_rate);
}

void cessb_set_release_ms(cessb_state_t *state, float release_ms) {
  state->lookahead.release_coeff = time_constant_to_coeff(release_ms, state->sample_rate);
}

int cessb_get_lookahead_samples(cessb_state_t *state) {
  return state->lookahead.lookahead_samples;
}

void cessb_debug_print_stats(cessb_state_t *state) {
  float peak_reduction_db = 0.0f;
  float avg_power_gain_db = 0.0f;

  cessb_get_stats(state, &peak_reduction_db, &avg_power_gain_db);

  printf("CESSB stats:\n");
  printf("  samples processed      : %ld\n", (long)state->sample_count);
  printf("  peak in (raw)          : %8.4f\n", state->peak_input);
  printf("  peak in (boosted)      : %8.4f\n", state->peak_input * CESSB_PRE_GAIN);
  printf("  peak after clip        : %8.4f\n", state->peak_after_clip);
  printf("  peak after overshoot   : %8.4f\n", state->peak_after_overshoot);
  printf("  peak out (post)        : %8.4f\n", state->peak_output);
  printf("  min limiter gain       : %8.4f\n", state->min_limiter_gain);
  printf("  peak reduction (dB)    : %8.2f dB\n", peak_reduction_db);
  printf("  avg power gain (dB)    : %8.2f dB\n", avg_power_gain_db);
}

// ============================================================================
// MAIN CESSB PROCESSING
// ============================================================================

void cessb_process(cessb_state_t *state, float *samples, int num_samples) {
  if (!state->enabled) {
    return;
  }

  int hilbert_delay_len = (HILBERT_TAPS / 2) + 1;

  for (int i = 0; i < num_samples; i++) {
    // measure input BEFORE pre-gain
    float abs_in = fabsf(samples[i]);
    if (abs_in > state->peak_input) {
      state->peak_input = abs_in;
    }
    state->average_power_in += samples[i] * samples[i];

    // apply pre-gain to boost small floats to working range
    float sample = samples[i] * CESSB_PRE_GAIN;
       
    // STAGE 1: Hilbert envelope detection
    float q = apply_fir_filter(hilbert_coeffs, state->hilbert_delay, &state->hilbert_index,
                               HILBERT_TAPS, sample);

    float i_delayed = get_delayed_sample(state->delay_line, &state->delay_index,
                                         hilbert_delay_len, sample);

    float envelope = sqrtf(i_delayed * i_delayed + q * q);

    // STAGE 2: Hard clip based on envelope
    float clipped;
    if (envelope > state->clip_level && envelope > 1e-10f) {
      float gain = state->clip_level / envelope;
      clipped = i_delayed * gain;
    } else {
      clipped = i_delayed;
    }

    float abs_clip = fabsf(clipped);
    if (abs_clip > state->peak_after_clip) {
      state->peak_after_clip = abs_clip;
    }

    // STAGE 3: Overshoot control filter
    float filtered =
        apply_fir_filter(overshoot_coeffs, state->overshoot_delay, &state->overshoot_index,
                         OVERSHOOT_FILTER_TAPS, clipped);

    float abs_filt = fabsf(filtered);
    if (abs_filt > state->peak_after_overshoot) {
      state->peak_after_overshoot = abs_filt;
    }

    // STAGE 4: Second Hilbert envelope detection
    float q2 = apply_fir_filter(hilbert_coeffs, state->hilbert2_delay,
                                &state->hilbert2_index, HILBERT_TAPS, filtered);
    float i2_delayed = get_delayed_sample(state->delay2_line, &state->delay2_index,
                                          hilbert_delay_len, filtered);
    float envelope2 = sqrtf(i2_delayed * i2_delayed + q2 * q2);

    // STAGE 5: Look-ahead limiter
    float limited = lookahead_limiter_process(&state->lookahead, i2_delayed, envelope2,
                                              state->envelope_limit);
    if (state->lookahead.current_gain < state->min_limiter_gain) {
      state->min_limiter_gain = state->lookahead.current_gain;
    }

    // STAGE 6: Post-limiter lowpass filter
    float output = apply_biquad_cascade(post_lpf_coeffs, state->post_lpf_state,
                                        POST_LPF_BIQUAD_STAGES, limited);

    state->sample_count++;

    // remove pre-gain before returning
    float final_output = output / CESSB_PRE_GAIN;

    // measure output AFTER removing pre-gain
    float abs_out = fabsf(final_output);
    if (abs_out > state->peak_output) {
      state->peak_output = abs_out;
    }
    state->average_power_out += final_output * final_output;
    samples[i] = final_output;
  }
}

void cessb_process_int32(cessb_state_t *state, int32_t *samples, int num_samples) {
  if (!state->enabled) {
    return;
  }

  float temp_buffer[1024];  // we get blocks of 1024 samples from sbitx tx_process()
  // convert int32 to float normalized to [-1, 1]
  for (int i = 0; i < num_samples; i++) {
    temp_buffer[i] = (float)samples[i] / 2147483648.0f;
  }

  // run CESSB processing (handles gain staging internally)
  cessb_process(state, temp_buffer, num_samples);

  // convert float back to int32
  for (int i = 0; i < num_samples; i++) {
    float out = temp_buffer[i] * 2147483647.0f;
    if (out > 2147483647.0f) {
      out = 2147483647.0f;
    }
    if (out < -2147483648.0f) {
      out = -2147483648.0f;
    }
    samples[i] = (int32_t)out;
  }  

  // Simple periodic stats print
  static unsigned long last_sample_count = 0;

  if (state->sample_count - last_sample_count >= 1000000UL) {
    last_sample_count = state->sample_count;
    cessb_debug_print_stats(state);
    cessb_reset_stats(state);
  }
}

// ============================================================================
// STATISTICS
// ============================================================================

void cessb_get_stats(cessb_state_t *state, float *peak_reduction_db,
                     float *avg_power_gain_db) {
  if (state->sample_count == 0) {
    *peak_reduction_db = 0.0f;
    *avg_power_gain_db = 0.0f;
    return;
  }

  if (state->peak_input > 1e-10f && state->peak_output > 1e-10f) {
    *peak_reduction_db = 20.0f * log10f(state->peak_output / state->peak_input);
  } else {
    *peak_reduction_db = 0.0f;
  }

  float avg_power_in = state->average_power_in / state->sample_count;
  float avg_power_out = state->average_power_out / state->sample_count;

  if (avg_power_in > 1e-10f && avg_power_out > 1e-10f) {
    *avg_power_gain_db = 10.0f * log10f(avg_power_out / avg_power_in);
  } else {
    *avg_power_gain_db = 0.0f;
  }
}

void cessb_reset_stats(cessb_state_t *state) {
  state->peak_input = 0.0f;
  state->peak_output = 0.0f;
  state->peak_after_clip = 0.0f;
  state->peak_after_overshoot = 0.0f;
  state->average_power_in = 0.0f;
  state->average_power_out = 0.0f;
  state->min_limiter_gain = 1.0f;
  // sample_count intentionally not reset - tracks total samples processed
}
