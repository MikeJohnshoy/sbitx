#include <stddef.h>

typedef struct {
  float i;
  float q;
} IQPair;

void cessb_reset(void);
IQPair* cessb_controlled_envelope(const IQPair* cessb_in);
