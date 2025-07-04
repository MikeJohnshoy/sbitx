#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // for memset

#define FFT_BLOCK_SIZE 256    // Set to your actual FFT block size
#define LOOKAHEAD_BLOCKS 8
#define IQ_PER_BLOCK (FFT_BLOCK_SIZE * 2)
#define BUFFER_SIZE (LOOKAHEAD_BLOCKS * IQ_PER_BLOCK)
#define LIMIT 0.95f

static struct {
    float iq_buffer[BUFFER_SIZE];
    int blocks_in_buffer;
    int initialized;
} cessb_lookahead_buf;

void cessb_lookahead_init(void) {
    memset(cessb_lookahead_buf.iq_buffer, 0, sizeof(cessb_lookahead_buf.iq_buffer));
    cessb_lookahead_buf.blocks_in_buffer = 0;
    cessb_lookahead_buf.initialized = 1;
}

int cessb_lookahead_process(const float *new_fft_block, float *limited_block) {
    if (!cessb_lookahead_buf.initialized) cessb_lookahead_init();

    if (cessb_lookahead_buf.blocks_in_buffer == LOOKAHEAD_BLOCKS) {
        cessb_envelope_limiter_lookahead(
            cessb_lookahead_buf.iq_buffer, cessb_lookahead_buf.iq_buffer,
            FFT_BLOCK_SIZE * LOOKAHEAD_BLOCKS,
            LIMIT,
            FFT_BLOCK_SIZE * LOOKAHEAD_BLOCKS
        );
        memcpy(limited_block, cessb_lookahead_buf.iq_buffer, sizeof(float) * IQ_PER_BLOCK);
        memmove(cessb_lookahead_buf.iq_buffer,
                cessb_lookahead_buf.iq_buffer + IQ_PER_BLOCK,
                sizeof(float) * IQ_PER_BLOCK * (LOOKAHEAD_BLOCKS - 1));
        cessb_lookahead_buf.blocks_in_buffer--;
        memcpy(cessb_lookahead_buf.iq_buffer + (cessb_lookahead_buf.blocks_in_buffer * IQ_PER_BLOCK),
               new_fft_block, sizeof(float) * IQ_PER_BLOCK);
        cessb_lookahead_buf.blocks_in_buffer++;
        return 1; // produced one output block
    } else {
        memcpy(cessb_lookahead_buf.iq_buffer + (cessb_lookahead_buf.blocks_in_buffer * IQ_PER_BLOCK),
               new_fft_block, sizeof(float) * IQ_PER_BLOCK);
        cessb_lookahead_buf.blocks_in_buffer++;
        return 0; // no output yet
    }
}

int cessb_lookahead_flush(float *limited_block) {
    if (cessb_lookahead_buf.blocks_in_buffer > 0) {
        cessb_envelope_limiter_lookahead(
            cessb_lookahead_buf.iq_buffer, cessb_lookahead_buf.iq_buffer,
            FFT_BLOCK_SIZE * cessb_lookahead_buf.blocks_in_buffer,
            LIMIT,
            FFT_BLOCK_SIZE * cessb_lookahead_buf.blocks_in_buffer
        );
        memcpy(limited_block, cessb_lookahead_buf.iq_buffer, sizeof(float) * IQ_PER_BLOCK);
        memmove(cessb_lookahead_buf.iq_buffer,
                cessb_lookahead_buf.iq_buffer + IQ_PER_BLOCK,
                sizeof(float) * IQ_PER_BLOCK * (cessb_lookahead_buf.blocks_in_buffer - 1));
        cessb_lookahead_buf.blocks_in_buffer--;
        return 1;
    }
    return 0;
}

// Simple soft clipper for gentle limiting
static float soft_clip(float x, float threshold) {
  if (fabsf(x) > threshold) {
    return copysignf(
        threshold + (fabsf(x) - threshold) / (1.0f + powf((fabsf(x) - threshold), 2)), x);
  }
  return x;
}

// CESSB envelope limiter with lookahead
// Working in frequency domain but using I and Q for real and imaginary
// in[]: input I/Q samples (interleaved: I, Q, I, Q, ...)
// out[]: output I/Q samples (same format)
// len: number of samples (I/Q pairs)
// limit: maximum allowed envelope (suggest: 0.9 to 0.99 for float)
// lookahead: number of samples (I/Q pairs) to look ahead (suggest: 8–32)
void cessb_envelope_limiter_lookahead(const float *in, float *out, size_t len, float limit,
                                      int lookahead) {
  if (lookahead < 1) lookahead = 1;
  if (lookahead > (int)len) lookahead = (int)len;

  // Create a buffer to hold lookahead samples' envelopes
  float *envelopes = (float *)malloc(sizeof(float) * len);
  if (!envelopes) return;  // Allocation failed

  // Compute envelope for each sample
  for (size_t i = 0; i < len; ++i) {
    float I = in[2 * i];
    float Q = in[2 * i + 1];
    envelopes[i] = sqrtf(I * I + Q * Q);
  }

  // Perform lookahead limiting
  for (size_t i = 0; i < len; ++i) {
    // Find the maximum envelope over the lookahead window
    float max_env = envelopes[i];
    size_t end = i + lookahead;
    if (end > len) end = len;
    for (size_t j = i; j < end; ++j) {
      if (envelopes[j] > max_env) max_env = envelopes[j];
    }

    float I = in[2 * i];
    float Q = in[2 * i + 1];
    float env = envelopes[i];

    // Compute gain to prevent exceeding the limit
    float gain = 1.0f;
    if (max_env > limit && max_env > 0.0f) gain = limit / max_env;

    // Apply soft clipping for smoother limiting
    I = soft_clip(I * gain, limit);
    Q = soft_clip(Q * gain, limit);

    out[2 * i] = I;
    out[2 * i + 1] = Q;
  }
  free(envelopes);
}
