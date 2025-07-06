#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // for memset

// *** tune for performance ***
#define LOOKAHEAD_SIZE 8   // minimum of 1
#define TARGET_PEAK 15.0f  // keep PA in linear range
// *** end of tunable values

typedef struct {
  float i;
  float q;
} IQPair;

// --- Static Buffers for Lookahead Processing ---
static IQPair lookahead_buffer[LOOKAHEAD_SIZE][FFT_BLOCK_SIZE];
static float peak_magnitudes[LOOKAHEAD_SIZE];  // peak magnitude found in each block
static int current_buffer_index = 0;
static int blocks_processed_count = 0;
static IQPair output_block[FFT_BLOCK_SIZE];

IQPair* cessb_lookahead_process(const IQPair* cessb_in) {
  // Find the peak magnitude of the incoming block and store the block itself
  float current_block_peak = 0.0f;
  for (int i = 0; i < FFT_BLOCK_SIZE; i++) {
    // Copy the sample into our circular buffer.
    lookahead_buffer[current_buffer_index][i] = cessb_in[i];
    // Calculate magnitude squared to avoid costly sqrt in the loop
    float mag_sq = (cessb_in[i].i * cessb_in[i].i) + (cessb_in[i].q * cessb_in[i].q);
    // Compare squared magnitudes
    if (mag_sq > (current_block_peak * current_block_peak)) {
      current_block_peak = sqrtf(mag_sq);
    }
  }
  peak_magnitudes[current_buffer_index] = current_block_peak;

  IQPair* block_to_return = NULL;

  // Check if we have received enough blocks to fill the lookahead window.
  // We can start processing once the buffer is full.
  if (blocks_processed_count >= LOOKAHEAD_SIZE - 1) {
    // The buffer is full, so we can process and return the oldest block.
    // In a circular buffer, the oldest block is the one after the one we just wrote to
    int oldest_block_index = (current_buffer_index + 1) % LOOKAHEAD_SIZE;
    // Find the largest peak across ALL blocks currently in the lookahead window.
    float largest_peak_found = 0.0f;
    for (int i = 0; i < LOOKAHEAD_SIZE; i++) {
      if (peak_magnitudes[i] > largest_peak_found) {
        largest_peak_found = peak_magnitudes[i];
      }
    }
    // Determine the scaling factor
    float scale_factor = 1.0f;  // Default to no scaling.
    if (largest_peak_found > TARGET_PEAK) {
      // If the largest peak found exceeds our target, calculate the
      // necessary scaling factor to bring it down to the target level.
      scale_factor = TARGET_PEAK / largest_peak_found;
    }
    // Apply the scaling factor to the OLDEST block and copy it to the output buffer.
    for (int i = 0; i < FFT_BLOCK_SIZE; i++) {
      output_block[i].i = lookahead_buffer[oldest_block_index][i].i * scale_factor;
      output_block[i].q = lookahead_buffer[oldest_block_index][i].q * scale_factor;
    }
    block_to_return = output_block;
  }

  // Update state for the next call.
  current_buffer_index = (current_buffer_index + 1) % LOOKAHEAD_SIZE;
  if (blocks_processed_count < LOOKAHEAD_SIZE) {
    blocks_processed_count++;
  }

  return block_to_return;
}
