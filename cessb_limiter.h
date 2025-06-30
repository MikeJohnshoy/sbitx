#pragma once
#include <stddef.h>
void cessb_envelope_limiter_lookahead(const float *in, float *out, size_t len, float limit, int lookahead);
