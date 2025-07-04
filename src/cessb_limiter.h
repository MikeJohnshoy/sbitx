#include <stddef.h>
void cessb_lookahead_init(void);
int cessb_lookahead_process(const float *new_fft_block, float *limited_block);
int cessb_lookahead_flush(float *limited_block);
void cessb_envelope_limiter_lookahead(const float *in, float *out, size_t len, float limit, int lookahead);
