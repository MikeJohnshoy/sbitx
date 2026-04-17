#pragma once
#include <stdint.h>

#define HPSDR_PKT_SIZE          1032
#define HPSDR_DISCOVERY_REPLY   64
#define SAMPLES_PER_PKT         126

// Packet types returned by hpsdr_classify()
#define PKT_UNKNOWN   0
#define PKT_DISCOVERY 1
#define PKT_START     2
#define PKT_STOP      3
#define PKT_EP2       4

typedef struct {
    int      mox;              // MOX bit from C0
    uint32_t freq;             // non-zero if freq command found
    float    iq[252];          // interleaved I,Q (up to 126 pairs)
    int      n_samples;        // number of IQ pairs extracted
} hpsdr_ep2_result_t;

int  hpsdr_classify(const uint8_t *buf, int len);
void hpsdr_build_discovery_reply(uint8_t *reply, int in_use);
int  hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result);