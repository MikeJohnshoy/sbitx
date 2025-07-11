#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cessb_limiter.h"

// *** Tune for performance ***
#define LOOKAHEAD_SIZE 4              // minimum of 1
#define LOOKAHEAD_TARGET_PEAK 10.0f  // desired final peak level for smooth scaling.
#define HARD_CLIP_LIMIT 10.5f        // slightly > LOOKAHEAD_TARGET_PEAK.
// *** End of tunable values ***

#define FFT_BLOCK_SIZE 2048

// Static buffers for lookahead processing
static IQPair lookahead_buffer[LOOKAHEAD_SIZE][FFT_BLOCK_SIZE];
static float peak_magnitudes[LOOKAHEAD_SIZE];  // Peak magnitude found in each RAW block
static int current_buffer_index = 0;
static int blocks_processed_count = 0;
static IQPair output_block[FFT_BLOCK_SIZE];
static IQPair zero_block[FFT_BLOCK_SIZE] = {0};  // Zero initialized

void cessb_reset(void) {
  memset(lookahead_buffer, 0, sizeof(lookahead_buffer));
  memset(peak_magnitudes, 0, sizeof(peak_magnitudes));
  current_buffer_index = 0;
  blocks_processed_count = 0;
  memset(output_block, 0, sizeof(output_block));
}

IQPair* cessb_controlled_envelope(const IQPair* cessb_in) {
  // --- analyze new input and populate the lookahead buffer ---
  float current_block_raw_peak = 0.0f;
  for (int i = 0; i < FFT_BLOCK_SIZE; i++) {
    // Store the original, unmodified sample in the lookahead buffer
    lookahead_buffer[current_buffer_index][i] = cessb_in[i];
    // Find the peak magnitude from the new, unclipped input signal
    float current_mag_sq = (cessb_in[i].i * cessb_in[i].i) + (cessb_in[i].q * cessb_in[i].q);
    if (current_mag_sq > (current_block_raw_peak * current_block_raw_peak)) {
      current_block_raw_peak = sqrtf(current_mag_sq);
    }
  }
  // Store the peak of the new block for future scaling decisions
  peak_magnitudes[current_buffer_index] = current_block_raw_peak;

  IQPair* block_to_return;
  if (blocks_processed_count >= LOOKAHEAD_SIZE - 1) {
    // lookahead buffer has been filled
    // --- calculate scaling factor based on the lookahead window ---
    // Find the largest peak across all blocks currently in the lookahead window
    float largest_peak_in_window = 0.0f;
    for (int i = 0; i < LOOKAHEAD_SIZE; i++) {
      if (peak_magnitudes[i] > largest_peak_in_window) {
        largest_peak_in_window = peak_magnitudes[i];
      }
    }
    // Determine the overall scaling factor to bring the window's max peak to our target
    float scale_factor = 1.0f;  // Default to no scaling.
    if (largest_peak_in_window > LOOKAHEAD_TARGET_PEAK) {
      scale_factor = LOOKAHEAD_TARGET_PEAK / largest_peak_in_window;
    }

    // --- apply scaling and final Clipping to the OLDEST block ---
    // which is next one in circular queue
    int oldest_block_index = (current_buffer_index + 1) % LOOKAHEAD_SIZE;
    for (int i = 0; i < FFT_BLOCK_SIZE; i++) {
      // Apply the calculated gentle scaling
      IQPair scaled_sample;
      scaled_sample.i = lookahead_buffer[oldest_block_index][i].i * scale_factor;
      scaled_sample.q = lookahead_buffer[oldest_block_index][i].q * scale_factor;

      // Apply the hard clipper as a final safety measure
      float scaled_mag =
          sqrtf((scaled_sample.i * scaled_sample.i) + (scaled_sample.q * scaled_sample.q));
      if (scaled_mag > HARD_CLIP_LIMIT) {
        float clip_factor = HARD_CLIP_LIMIT / scaled_mag;
        output_block[i].i = scaled_sample.i * clip_factor;
        output_block[i].q = scaled_sample.q * clip_factor;
      } else {
        output_block[i] = scaled_sample;
      }
    }

    // FUTURE WORK: Optional linear-phase FIR filter would go here.

    block_to_return = output_block;
  } else {
    // Not enough lookahead data yet; return silence.
    block_to_return = zero_block;
  }

  // Update state for the next call
  current_buffer_index = (current_buffer_index + 1) % LOOKAHEAD_SIZE;
  if (blocks_processed_count < LOOKAHEAD_SIZE) {
    blocks_processed_count++;
  }
  return block_to_return;
}
