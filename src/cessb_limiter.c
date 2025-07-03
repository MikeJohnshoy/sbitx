#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // for memset

// create a log for CESSB debug purposes only
// this log will be BAD for performance and should be deleted from production
// code
void cessb_log_data(float max_env, float max_mag) {
  FILE *log = fopen("cessb_limiter.log", "a");
  if (log) {
    fprintf(log, "max_env out of limiter: %f, max_env out of soft clipper: %f\n", max_env,
            max_mag);
    fclose(log);
  }
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

/*
  // for DEVELOPMENT PURPOSES ONLY
  // remove from production code
  // Find maximum envelope out of look ahead limiter
  float max_env = envelopes[0];
  for (size_t i = 1; i < len; ++i) {
    if (envelopes[i] > max_env) max_env = envelopes[i];
  }

  // Find maximum output magnitude out of soft clipper
  float max_out_mag = 0.0f;
  for (size_t i = 0; i < len; ++i) {
    float mag = sqrtf(out[2 * i] * out[2 * i] + out[2 * i + 1] * out[2 * i + 1]);
    if (mag > max_out_mag) max_out_mag = mag;
  }

  // Log the values
  cessb_log_data(max_env, max_out_mag);
*/

  free(envelopes);
}
