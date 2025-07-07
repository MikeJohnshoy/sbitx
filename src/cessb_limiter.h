#include <stddef.h>
int cessb_lookahead_process(const IQPair* cessb_in);

typedef struct {
  float i;
  float q;
} IQPair;
