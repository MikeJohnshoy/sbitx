#ifndef HPSDR_P1_H
#define HPSDR_P1_H

#include <stdint.h>

#define HPSDR_PORT 1024
#define HPSDR_PKT_SIZE 1032
#define SAMPLES_PER_PACKET 126  // pick one!
#define SAMPLES_PER_PKT    126

// Packet types returned by hpsdr_classify()
#define PKT_UNKNOWN   0
#define PKT_DISCOVERY 1
#define PKT_START     2
#define PKT_STOP      3
#define PKT_EP2       4
#define HPSDR_DISCOVERY_REPLY   60

typedef struct {
    int      mox;
    uint32_t freq;                      // RX NCO freq (addr 0x02)
    uint32_t tx_freq;                   // TX NCO freq (addr 0x01), 0 if not present
    int      n_samples;
    float    iq[SAMPLES_PER_PKT * 2];  // ← add this: interleaved I,Q pairs
} hpsdr_ep2_result_t;

int  hpsdr_init(void);
void hpsdr_stop(void);
void hpsdr_send_iq(double *i_samples, double *q_samples, int n);
void hpsdr_poll(void);
int  hpsdr_is_connected(void);

// --- TX IQ from remote SDR client (e.g., SDRConsole) ---
// Returns 1 if a remote app is actively sending TX IQ data
int  hpsdr_tx_iq_active(void);

// Retrieve up to max_samples of 96kHz-upsampled TX IQ.
// Returns the number of sample pairs actually written.
int  hpsdr_get_tx_iq(double *out_i, double *out_q, int max_samples);

#endif // HPSDR_P1_H
