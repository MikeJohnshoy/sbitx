#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h> // for memset

// Simple soft clipper for gentle limiting
static float soft_clip(float x, float threshold) {
    if (fabsf(x) > threshold) {
        return copysignf(threshold + (fabsf(x) - threshold) / (1.0f + powf((fabsf(x) - threshold), 2)), x);
    }
    return x;
}

// CESSB envelope limiter with lookahead
// in[]: input I/Q samples (interleaved: I, Q, I, Q, ...)
// out[]: output I/Q samples (same format)
// len: number of samples (I/Q pairs)
// limit: maximum allowed envelope (suggest: 0.9 to 0.99 for float)
// lookahead: number of samples (I/Q pairs) to look ahead (suggest: 8–32)
void cessb_envelope_limiter_lookahead(
    const float *in, float *out, size_t len, float limit, int lookahead
) {
    if (lookahead < 1) lookahead = 1;
    if (lookahead > (int)len) lookahead = (int)len;

    // Create a buffer to hold lookahead samples' envelopes
    float *envelopes = (float*)malloc(sizeof(float) * len);
    if (!envelopes) return; // Allocation failed

    // Compute envelope for each sample
    for (size_t i = 0; i < len; ++i) {
        float I = in[2*i];
        float Q = in[2*i+1];
        envelopes[i] = sqrtf(I*I + Q*Q);
    }

    // Perform lookahead limiting
    for (size_t i = 0; i < len; ++i) {
        // Find the maximum envelope over the lookahead window
        float max_env = envelopes[i];
        size_t end = i + lookahead;
        if (end > len) end = len;
        for (size_t j = i; j < end; ++j) {
            if (envelopes[j] > max_env)
                max_env = envelopes[j];
        }

        float I = in[2*i];
        float Q = in[2*i+1];
        float env = envelopes[i];

        // Compute gain to prevent exceeding the limit
        float gain = 1.0f;
        if (max_env > limit && max_env > 0.0f)
            gain = limit / max_env;

        // Optionally, use soft clipping for smoother limiting
        I = soft_clip(I * gain, limit);
        Q = soft_clip(Q * gain, limit);

        out[2*i]   = I;
        out[2*i+1] = Q;
    }

    free(envelopes);
}
