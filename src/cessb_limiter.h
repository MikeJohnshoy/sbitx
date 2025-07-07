#include <stddef.h>

typedef struct {
  float i;
  float q;
} IQPair;

IQPair* cessb_lookahead_process(const IQPair* cessb_in);
