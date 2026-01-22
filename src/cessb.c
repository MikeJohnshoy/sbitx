// CESSB (Controlled Envelope Single Sideband) Processing Implementation
//
// Implementation based on the technique described by David Hershberger, W9GR
// in QEX November/December 2014: "Controlled Envelope Single Sideband".
//
// This chain follows the three-stage CESSB summary:
// 1) Prefilter and peak-limit the audio input before the Hilbert transform.
// 2) Baseband “RF” clipping on the analytic I/Q via magnitude clipping.
// 3) Overshoot compensation using a look-ahead envelope limiter with
//    attack/release smoothing, plus a hard envelope cap, and final post-LPF
//    and soft-clip guard on the real output.
//
// Concept: get audio samples in blocks, apply CESSB process, return them.
// - Build analytic I and Q and envelope for every sample in CURRENT input block.
// - Apply Stage 1 (prefilter + peak limit on audio) before Hilbert/delay.
// - Apply Stage 2 (magnitude clip on analytic I/Q) to reduce Hilbert overshoot.
// - Apply Stage 3 (overshoot compensation): compute per-sample gain with a
//   look-ahead window (LA samples spanning previous+current), smooth gain
//   (attack/release), hard-cap envelope if needed, post-filter the real part,
//   and guard soft-clip on the final output.
// - Process each sample of the PREVIOUS block sequentially with its gain and
//   guards. Output the processed PREVIOUS block.
// - Shift the pipeline: the CURRENT block becomes the PREVIOUS block.
//
// Assumptions:
// - num_samples <= 1024 (matches sbitx tx_process block sizing)
// - sample_rate is constant (96kHz)
//
// added by Mike KB2ML and Bob KD8CGH

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <time.h>  // for debug stats only
#include "cessb.h"

// Tunables
#define CESSB_AUDIO_PEAK_LIMIT   0.9f     // Stage 1 peak limit (pre-Hilbert)
#define CESSB_AUDIO_PRE_FC       4000.0f  // Stage 1 LPF cutoff (~4 kHz)
#define CESSB_RF_CLIP_LEVEL      1.05f    // Stage 2 complex-envelope clip
#define CESSB_LA_SAMPLES 96       // look-ahead window length in samples (≈1ms @ 96kHz)
#define CESSB_BLOCK_MAX 1024      // expected maximum block size
#define CESSB_RING_MAX (CESSB_LA_SAMPLES + CESSB_BLOCK_MAX)
#define CESSB_GAIN_ATTACK_MS 0.05f  // gain smoothing time constants (ms)
#define CESSB_GAIN_RELEASE_MS 15.0f
#define CESSB_OUTPUT_GUARD 0.99f  // final soft-clip guard threshold
#define HILBERT_DELAY_LEN (HILBERT_TAPS / 2)  

// Debug control: gap-based transmission detection
static struct timespec debug_last_ts = {0, 0};
static int debug_have_ts = 0;
#define DEBUG_IDLE_GAP_S 0.25  // treat gaps > 250 ms as a new transmission

// global CESSB state
int cessb_enabled = 0;
cessb_state_t cessb_processor;

// Sliding-window look-ahead ring buffers (cross-block continuity)
static float la_buf_i[CESSB_RING_MAX];
static float la_buf_q[CESSB_RING_MAX];
static float la_buf_env[CESSB_RING_MAX];
static int   la_buf_head = 0;   // index of oldest sample
static int   la_buf_len  = 0;   // number of valid samples in buffer
// Smoothed gain state (continuous across blocks)
static float la_gain = 1.0f;

static inline void cessb_la_block_reset(void) {
  la_buf_head = 0;
  la_buf_len = 0;
  la_gain = 1.0f;
  memset(la_buf_i, 0, sizeof(la_buf_i));
  memset(la_buf_q, 0, sizeof(la_buf_q));
  memset(la_buf_env, 0, sizeof(la_buf_env));
}

// Hilbert transform coefficients (127-tap equiripple FIR, Type IV)
// Design (scipy.signal.remez): 
//   Sample rate:        96000 Hz
//   Passband:           250 – 3500 Hz  
//   Transition width:   ~200 Hz
//   Passband ripple:    < 0.1 dB
//   Stopband atten:     > 50 dB
//   Group delay:        63 samples
// Antisymmetric (Type IV FIR): h[n] = -h[N-1-n], all taps used
static const float hilbert_coeffs[HILBERT_TAPS] = {
    -0.037764f, -0.010794f, -0.011979f, -0.012843f, -0.013441f, -0.013775f, -0.013656f, -0.013168f,
    -0.012250f, -0.010955f, -0.009276f, -0.007232f, -0.004877f, -0.002271f,  0.000492f,  0.003327f,
     0.006151f,  0.008874f,  0.011418f,  0.013688f,  0.015611f,  0.017117f,  0.018165f,  0.018728f,
     0.018798f,  0.018390f,  0.017542f,  0.016312f,  0.014784f,  0.013051f,  0.011221f,  0.009406f,
     0.007727f,  0.006304f,  0.005253f,  0.004678f,  0.004664f,  0.005279f,  0.006563f,  0.008531f,
     0.011168f,  0.014425f,  0.018227f,  0.022469f,  0.027025f,  0.031746f,  0.036469f,  0.041019f,
     0.045218f,  0.048889f,  0.051866f,  0.054001f,  0.055164f,  0.055256f,  0.054207f,  0.051985f,
     0.048596f,  0.044082f,  0.038523f,  0.032030f,  0.024749f,  0.016852f,  0.008533f,  0.000000f,
    -0.008533f, -0.016852f, -0.024749f, -0.032030f, -0.038523f, -0.044082f, -0.048596f, -0.051985f,
    -0.054207f, -0.055256f, -0.055164f, -0.054001f, -0.051866f, -0.048889f, -0.045218f, -0.041019f,
    -0.036469f, -0.031746f, -0.027025f, -0.022469f, -0.018227f, -0.014425f, -0.011168f, -0.008531f,
    -0.006563f, -0.005279f, -0.004664f, -0.004678f, -0.005253f, -0.006304f, -0.007727f, -0.009406f,
    -0.011221f, -0.013051f, -0.014784f, -0.016312f, -0.017542f, -0.018390f, -0.018798f, -0.018728f,
    -0.018165f, -0.017117f, -0.015611f, -0.013688f, -0.011418f, -0.008874f, -0.006151f, -0.003327f,
    -0.000492f,  0.002271f,  0.004877f,  0.007232f,  0.009276f,  0.010955f,  0.012250f,  0.013168f,
     0.013656f,  0.013775f,  0.013441f,  0.012843f,  0.011979f,  0.010794f,  0.037764f
};

// DSP helpers
static inline float soft_clip(float sample, float threshold) {
  if (sample > threshold) {
    return threshold + (sample - threshold) / (1.0f + fabsf(sample - threshold));
  } else if (sample < -threshold) {
    return -threshold + (sample + threshold) / (1.0f + fabsf(sample + threshold));
  }
  return sample;
}

static float hilbert_transform(cessb_state_t *state, float input) {
  state->hilbert_delay[state->hilbert_index] = input;
  float output = 0.0f;
  int idx = state->hilbert_index;
  for (int i = 0; i < HILBERT_TAPS; i++) {
    output += state->hilbert_delay[idx] * hilbert_coeffs[i];
    idx--;
    if (idx < 0) idx = HILBERT_TAPS - 1;
  }
  state->hilbert_index++;
  if (state->hilbert_index >= HILBERT_TAPS) state->hilbert_index = 0;
  return output;
}

// get delayed sample (compensates for Hilbert transform group delay)
static float delay_sample(cessb_state_t *state, float input) {
  float output = state->delay_line[state->delay_index];
  state->delay_line[state->delay_index] = input;
  state->delay_index++;
  if (state->delay_index >= HILBERT_DELAY_LEN) state->delay_index = 0;
  return output;
}

// State helpers
void cessb_init(cessb_state_t *state) {
  memset(state, 0, sizeof(cessb_state_t));
  state->enabled = 0;
  state->clip_level = CESSB_CLIP_LEVEL;
  state->envelope_limit = CESSB_ENVELOPE_LIMIT;
  state->hilbert_index = 0;
  state->delay_index = 0;
  cessb_la_block_reset();
}

void cessb_set_enabled(cessb_state_t *state, int enabled) {
  state->enabled = enabled ? 1 : 0;
  if (enabled) {
    // reset filter states when enabling
    memset(state->hilbert_delay, 0, sizeof(state->hilbert_delay));
    memset(state->delay_line, 0, sizeof(state->delay_line));
    cessb_la_block_reset();
    cessb_reset_stats(state);

    // Force next block to be treated as a new transmission
    debug_have_ts = 0;
    debug_last_ts.tv_sec = 0;
    debug_last_ts.tv_nsec = 0;
  } else {
    // On disable, also force next enable to look like a fresh transmission
    debug_have_ts = 0;
    debug_last_ts.tv_sec = 0;
    debug_last_ts.tv_nsec = 0;
  }
}

void cessb_set_clip_level(cessb_state_t *state, float level) {
  if (level > 0.0f && level <= 1.0f) {
    state->clip_level = level;
  }
}

void cessb_set_envelope_limit(cessb_state_t *state, float limit) {
  if (limit > 0.0f && limit <= 2.0f) {
    state->envelope_limit = limit;
  }
}

int cessb_is_enabled(cessb_state_t *state) {
  return state->enabled;
}

// main process (float path)
void cessb_process(cessb_state_t *state, float *samples, int num_samples,
                   float sample_rate) {
  if (!state->enabled || num_samples <= 0) return;
  if (num_samples > CESSB_BLOCK_MAX) return;

  // Precompute constants
  const float post_alpha =
      expf(-2.0f * (float)M_PI * CESSB_POST_LPF_CUTOFF / sample_rate);
  const float att_a =
      expf(-(1.0f / (0.001f * CESSB_GAIN_ATTACK_MS)) / sample_rate);
  const float rel_a =
      expf(-(1.0f / (0.001f * CESSB_GAIN_RELEASE_MS)) / sample_rate);
  const float audio_alpha =
      expf(-2.0f * (float)M_PI * CESSB_AUDIO_PRE_FC / sample_rate);

  // Stage buffers
  float cur_i[CESSB_BLOCK_MAX];
  float cur_q[CESSB_BLOCK_MAX];
  float cur_env[CESSB_BLOCK_MAX];
  float out_block[CESSB_BLOCK_MAX];
  memset(out_block, 0, sizeof(float) * (size_t)num_samples);

  // Stats accumulators
  float peak_in = 0.0f, peak_out = 0.0f;
  float sum_sq_in = 0.0f, sum_sq_out = 0.0f;

  // Stage 1: audio prefilter + peak limit (stateful one-pole)
  static float audio_lpf_state = 0.0f;
  for (int i = 0; i < num_samples; i++) {
    float sample = samples[i];

    // Prefilter
    float audio_prefilt =
        (1.0f - audio_alpha) * sample + audio_alpha * audio_lpf_state;
    audio_lpf_state = audio_prefilt;

    // Peak limit
    if (audio_prefilt > CESSB_AUDIO_PEAK_LIMIT)  audio_prefilt = CESSB_AUDIO_PEAK_LIMIT;
    if (audio_prefilt < -CESSB_AUDIO_PEAK_LIMIT) audio_prefilt = -CESSB_AUDIO_PEAK_LIMIT;

    sample = audio_prefilt;  // replace input with Stage 1 output

    // Track input stats
    float abs_in = fabsf(sample);
    if (abs_in > peak_in) peak_in = abs_in;
    sum_sq_in += sample * sample;

    // Analytic signal
    float q  = hilbert_transform(state, sample);
    float ii = delay_sample(state, sample);

    cur_i[i] = ii;
    cur_q[i] = q;
    cur_env[i] = sqrtf(ii * ii + q * q);
  }

  // Stage 2: magnitude clip on analytic I/Q
  for (int i = 0; i < num_samples; i++) {
    float mag = hypotf(cur_i[i], cur_q[i]);
    if (mag > CESSB_RF_CLIP_LEVEL) {
      float scale = CESSB_RF_CLIP_LEVEL / (mag + 1e-9f); // avoid div/0
      cur_i[i] *= scale;
      cur_q[i] *= scale;
      mag = CESSB_RF_CLIP_LEVEL;
    }
    cur_env[i] = mag;
  }

  // Stage 3: sample-accurate look-ahead limiter + hard cap + post-LPF + guard
  // Uses a sliding window of length CESSB_LA_SAMPLES; outputs are delayed by (LA-1) samples.
  int outputs_emitted = 0;
  for (int n = 0; n < num_samples; n++) {
    // Push current sample into ring
    int pos = (la_buf_head + la_buf_len) % CESSB_RING_MAX;
    la_buf_i[pos] = cur_i[n];
    la_buf_q[pos] = cur_q[n];
    la_buf_env[pos] = cur_env[n];
    if (la_buf_len < CESSB_RING_MAX) la_buf_len++;

    // Emit once we have at least LA samples buffered
    if (la_buf_len >= CESSB_LA_SAMPLES) {
      // Compute max envelope over the last LA samples (O(LA); LA=96)
      float env_max = 0.0f;
      for (int k = 0; k < CESSB_LA_SAMPLES; k++) {
        int idx = (pos - k + CESSB_RING_MAX) % CESSB_RING_MAX;
        float ev = la_buf_env[idx];
        if (ev > env_max) env_max = ev;
      }

      float g_target = 1.0f;
      if (env_max > state->envelope_limit && env_max > 1e-9f) {
        g_target = state->envelope_limit / env_max;
      }

      // Smooth gain per-sample
      if (g_target < la_gain) {
        la_gain = (1.0f - att_a) * g_target + att_a * la_gain;
      } else {
        la_gain = (1.0f - rel_a) * g_target + rel_a * la_gain;
      }

      // Oldest sample in the window is at head; that is the one we output now
      int out_idx = la_buf_head;
      float i_limited = la_buf_i[out_idx] * la_gain;
      float q_limited = la_buf_q[out_idx] * la_gain;

      // Hard envelope cap
      float env2 = sqrtf(i_limited * i_limited + q_limited * q_limited);
      if (env2 > state->envelope_limit && env2 > 0.0001f) {
        float scale = state->envelope_limit / env2;
        i_limited *= scale;
        q_limited *= scale;
      }

      float output_unfiltered = i_limited;  // transmit real part
      state->post_lpf_state = (1.0f - post_alpha) * output_unfiltered +
                              post_alpha * state->post_lpf_state;
      float output = soft_clip(state->post_lpf_state, CESSB_OUTPUT_GUARD);

      // Output slot is delayed by (LA-1) samples relative to current n
      int out_slot = n - (CESSB_LA_SAMPLES - 1);
      if (out_slot >= 0 && out_slot < num_samples) {
        out_block[out_slot] = output;
      }

      // Pop head
      la_buf_head = (la_buf_head + 1) % CESSB_RING_MAX;
      la_buf_len--;

      // Stats
      float abs_out = fabsf(output);
      if (abs_out > peak_out) peak_out = abs_out;
      sum_sq_out += output * output;
      outputs_emitted++;
    }
  }

  // Write outputs (leading (LA-1) samples of the first call will be zero, not a whole block)
  for (int i = 0; i < num_samples; i++) {
    samples[i] = out_block[i];
  }

  // update statistics
  state->peak_input  = 0.9f * state->peak_input  + 0.1f * peak_in;
  state->peak_output = 0.9f * state->peak_output + 0.1f * peak_out;
  state->average_power_in  =
      0.95f * state->average_power_in + 0.05f * (sum_sq_in / num_samples);
  if (outputs_emitted > 0) {
    state->average_power_out =
        0.95f * state->average_power_out + 0.05f * (sum_sq_out / outputs_emitted);
  }
}

// called from tx_process in sbitx.c
// gets block of integer samples, converts to float, processes, and converts back to int
void cessb_process_int32(cessb_state_t *state, int32_t *samples,
                         int num_samples, float sample_rate) {
  // Debug session control (per transmission)
  static int debug_active = 0;       // true while collecting/printing this transmission
  static int debug_blocks_left = 0;  // countdown of blocks to process for debug
  static float cessb_debug_accum_s = 0.0f;

  if (!state->enabled || num_samples <= 0 || num_samples > 1024) {
    printf("CESSB: unexpected input, num_samples: %d\n", num_samples);
    return;
  }

  // Detect idle gap to infer a new transmission
  int new_transmission = 0;
  {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) == 0) {
      if (debug_have_ts) {
        double dt = (now.tv_sec - debug_last_ts.tv_sec)
                    + (now.tv_nsec - debug_last_ts.tv_nsec) / 1e9;
        if (dt > DEBUG_IDLE_GAP_S) {
          new_transmission = 1;
        }
      } else {
        // First call ever
        new_transmission = 1;
      }
      debug_last_ts = now;
      debug_have_ts = 1;
    }
  }

  // Start debug collection on each detected transmission start
  if (new_transmission) {
    cessb_reset_stats(state);   // clear accumulated data before starting
    debug_active = 1;
    debug_blocks_left = 500;    // process 500 blocks, then stop
    cessb_debug_accum_s = 0.0f;
  }

  const float scale_to_float = 1.0f / 2000000000.0f;
  const float scale_to_int32 = 2000000000.0f;

  float float_samples[1024];

  for (int i = 0; i < num_samples; i++) {
    float_samples[i] = samples[i] * scale_to_float;
  }

  cessb_process(state, float_samples, num_samples, sample_rate);

  for (int i = 0; i < num_samples; i++) {
    float clamped = float_samples[i];
    if (clamped > 1.0f) clamped = 1.0f;
    if (clamped < -1.0f) clamped = -1.0f;
    samples[i] = (int32_t)(clamped * scale_to_int32);
  }

  // CESSB demo/debug stats:
  // - runs at the start of each transmission (detected via idle gap)
  // - clears stats before starting
  // - processes 500 blocks then stops until the next transmission
  if (debug_active) {
    cessb_debug_accum_s += (float)num_samples / sample_rate;

    if (cessb_debug_accum_s >= 5.0f) {
      cessb_debug_accum_s -= 5.0f;  // keep residual, handle non-integer multiples

      float peak_red_db = 0.0f;
      float avg_gain_db = 0.0f;
      cessb_get_stats(state, &peak_red_db, &avg_gain_db);

      // Note: peak_red_db will be negative if output peak < input peak.
      printf("CESSB STATS: peak_in=%.3f peak_out=%.3f peak_reduction=%.2f dB "
             "avg_power_in=%.6f avg_power_out=%.6f avg_power_gain=%.2f dB\n",
             state->peak_input,
             state->peak_output,
             peak_red_db,
             state->average_power_in,
             state->average_power_out,
             avg_gain_db);
    }

    // Stop collecting after 500 blocks in this transmission
    if (--debug_blocks_left <= 0) {
      debug_active = 0;
    }
  }
}

// stats
void cessb_get_stats(cessb_state_t *state, float *peak_reduction_db,
                     float *avg_power_gain_db) {
  if (peak_reduction_db) {
    if (state->peak_input > 0.0001f && state->peak_output > 0.0001f) {
      *peak_reduction_db =
          20.0f * log10f(state->peak_output / state->peak_input);
    } else {
      *peak_reduction_db = 0.0f;
    }
  }

  if (avg_power_gain_db) {
    if (state->average_power_in > 0.000001f &&
        state->average_power_out > 0.000001f) {
      *avg_power_gain_db =
          10.0f * log10f(state->average_power_out / state->average_power_in);
    } else {
      *avg_power_gain_db = 0.0f;
    }
  }
}

void cessb_reset_stats(cessb_state_t *state) {
  state->peak_input = 0.0f;
  state->peak_output = 0.0f;
  state->average_power_in = 0.0f;
  state->average_power_out = 0.0f;
}

