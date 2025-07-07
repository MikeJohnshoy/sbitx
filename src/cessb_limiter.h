#include <stddef.h>

typedef struct {
  float i;
  float q;
} IQPair;

IQPair* cessb_controlled_envelope(const IQPair* cessb_in);
