#ifndef HPSDR_P1_H
#define HPSDR_P1_H

#include <stdint.h>

#define HPSDR_PORT 1024
#define HPSDR_PKT_SIZE 1032
#define SAMPLES_PER_PACKET 126

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
