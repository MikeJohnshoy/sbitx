#include <stddef.h>

typedef struct {
  float i;
  float q;
} IQPair;

int cessb_lookahead_process(const IQPair* cessb_in);
