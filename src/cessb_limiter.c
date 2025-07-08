#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // for memset

// *** tune for performance ***
#define LOOKAHEAD_SIZE 3   // minimum of 1
#define TARGET_PEAK 150.0f // keep PA in linear range
// *** end of tunable values

typedef struct {
  float i;
  float q;
} IQPair;

#define FFT_BLOCK_SIZE 2048

// --- Static Buffers for Lookahead Processing ---
static IQPair lookahead_buffer[LOOKAHEAD_SIZE][FFT_BLOCK_SIZE];
static float peak_magnitudes[LOOKAHEAD_SIZE];  // peak magnitude found in each block
static int current_buffer_index = 0;
static int blocks_processed_count = 0;
static IQPair output_block[FFT_BLOCK_SIZE];

// This function performs per-sample clipping and lookahead-based
// block scaling, followed by a (not implemented) linear-phase FIR filter.
IQPair* cessb_controlled_envelope(const IQPair* cessb_in) {
  // Initialize current block peak for the lookahead stage
  float current_block_peak = 0.0f;

  // Baseband Envelope Clipper Stage (per sample)
  // This loop processes the incoming block, applying immediate clipping
  // if any sample's magnitude exceeds TARGET_PEAK, and then stores
  // the (potentially clipped) sample into the lookahead buffer.
  // It also calculates the peak magnitude for this specific block
  // from the clipped data, which will be used in the lookahead window.
  for (int i = 0; i < FFT_BLOCK_SIZE; i++) {
    float current_mag = sqrtf((cessb_in[i].i * cessb_in[i].i) + (cessb_in[i].q * cessb_in[i].q));

    if (current_mag > TARGET_PEAK) {  // Use TARGET_PEAK for immediate clipping
      float clip_factor = TARGET_PEAK / current_mag;
      // Apply clipping and store into the lookahead buffer
      lookahead_buffer[current_buffer_index][i].i = cessb_in[i].i * clip_factor;
      lookahead_buffer[current_buffer_index][i].q = cessb_in[i].q * clip_factor;
    } else {
      // No clipping needed, just copy the sample
      lookahead_buffer[current_buffer_index][i] = cessb_in[i];
    }

    // Now, calculate the peak for the *lookahead* stage from the potentially clipped signal
    float mag_sq_after_clip =
        (lookahead_buffer[current_buffer_index][i].i *
         lookahead_buffer[current_buffer_index][i].i) +
        (lookahead_buffer[current_buffer_index][i].q * lookahead_buffer[current_buffer_index][i].q);

    // Update the peak for the current block based on the clipped values
    if (mag_sq_after_clip > (current_block_peak * current_block_peak)) {
      current_block_peak = sqrtf(mag_sq_after_clip);
    }
  }
  // Store the peak magnitude of the current (potentially clipped) block
  peak_magnitudes[current_buffer_index] = current_block_peak;

  // Check if we have received enough blocks to fill the lookahead window.
  // We can start processing once the buffer is full
  IQPair* block_to_return = NULL;
  if (blocks_processed_count >= LOOKAHEAD_SIZE - 1) {
    // The buffer is full, so we can process and return the oldest block.
    // In a circular buffer, the oldest block is the one after the one we just wrote to
    int oldest_block_index = (current_buffer_index + 1) % LOOKAHEAD_SIZE;

    // Find the largest peak across ALL blocks currently in the lookahead window.
    // This largest_peak_found is based on the *already clipped* block peaks.
    float largest_peak_found = 0.0f;
    for (int i = 0; i < LOOKAHEAD_SIZE; i++) {
      if (peak_magnitudes[i] > largest_peak_found) {
        largest_peak_found = peak_magnitudes[i];
      }
    }

    // Determine the overall scaling factor for the lookahead window
    float scale_factor = 1.0f;  // Default to no scaling.
    if (largest_peak_found > TARGET_PEAK) {
      // If the largest peak found across the lookahead window exceeds our target,
      // calculate the necessary scaling factor to bring it down to the target level.
      scale_factor = TARGET_PEAK / largest_peak_found;
    }

    // Apply the scaling factor to the OLDEST block and copy it to the output buffer
    // (in circular buffer the oldest block is always the next one to be output)
    for (int i = 0; i < FFT_BLOCK_SIZE; i++) {
      output_block[i].i = lookahead_buffer[oldest_block_index][i].i * scale_factor;
      output_block[i].q = lookahead_buffer[oldest_block_index][i].q * scale_factor;
    }

    // 
    // FUTURE WORK: Add an optional linear-phase FIR filter
    // if someone thinks we need it
    //
    
    block_to_return = output_block;
  }

  // Update state for the next call.
  current_buffer_index = (current_buffer_index + 1) % LOOKAHEAD_SIZE;
  if (blocks_processed_count < LOOKAHEAD_SIZE) {
    blocks_processed_count++;
  }

  return block_to_return;
}
