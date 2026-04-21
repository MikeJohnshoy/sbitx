// hpsdr_p1.c — HPSDR Protocol 1 interface for sBitx
//
// Provide the interface between the sbitx and an external SDR app using hpsdr Protocol 1
// Major functions are:
//   Signal processing and buffering:
//    - manage a data buffer to prevent dropping data between the sbitx and external app
//    - perform data rate conversion between sbitx internal 96k samples per second
//      and the external SDR app 48k samples per second
//   HPSDR Protocol 1:
//    - identify packet types
//    - add or extract I and Q and other controls and data, and copy in and out of buffer
//   Stater translation
//    - coordinate state between sBitx and external SDR app (T/R switch, freq, gain settings,
//      etc.
//   Initialization control and shutdown
//    - sbitx.c needs to start and stop and get status on this interface

#include "hpsdr_p1.h"
#include <arpa/inet.h>
#include <gtk/gtk.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

// function prototypes
// Signal Processing & Buffering (Section 1)
static unsigned long millis_now(void);
static void flush_tx_ring(void);
static void tx_iq_push_48k(double i_val, double q_val);
// Public: hpsdr_tx_iq_active, hpsdr_get_tx_iq, hpsdr_send_iq (defined in .h)

// HPSDR Protocol 1 Implementation (Section 2)
static int hpsdr_classify(const uint8_t *buf, int len);
static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use);
static void build_and_send_packet(void);
static int hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result);
static void reset_all_tx_state(void);
static void handle_command(uint8_t *buf, int len, struct sockaddr_in *sender);

// Initialization, Control & Shutdown (Section 3)
static gboolean hpsdr_tr_idle(gpointer data);
static gboolean hpsdr_watchdog(gpointer data);
static void *hpsdr_poll_thread(void *arg);
// Public: hpsdr_init, hpsdr_stop, hpsdr_is_connected, hpsdr_poll (defined in .h)

// --- Configuration & Statics ---
#define HPSDR_PORT 1024
#define HPSDR_PKT_SIZE 1032
#define SAMPLES_PER_PACKET 126
#define TX_SOFT 2

static int hpsdr_sock = -1;
static struct sockaddr_in stream_dest;
static volatile int client_active = 0;
static volatile int running = 0;
static uint32_t tx_seq = 0;
static pthread_t poll_thread;
static volatile int tr_pending = 0;

// Externs for sBitx core interaction
extern void remote_execute(char *command);
extern int freq_hdr;
extern int in_tx;
extern void tr_switch(int tx_on);
extern void tx_on(int trigger);
extern void tx_off(void);

// =============================================================================
// SIGNAL PROCESSING & BUFFERING
// =============================================================================

// IQ accumulation buffer for 126 samples (48kHz)
static double iq_buf_i[SAMPLES_PER_PACKET];
static double iq_buf_q[SAMPLES_PER_PACKET];
static int iq_buf_count = 0;
static double hpsdr_iq_gain = 30.0; // add gain to I and Q data going out
static double hpsdr_tx_gain = 30.0; // add gain to I and Q coming from external SDR

// Filter state for the 24kHz LPF (stores the last sample of the previous block)
static double last_i = 0.0;
static double last_q = 0.0;

// TX IQ ring buffer statics
#define TX_IQ_RING_SIZE 8192 // must be power of 2, ~85 ms at 96 kHz
#define TX_IQ_RING_MASK (TX_IQ_RING_SIZE - 1)
static double tx_iq_ring_i[TX_IQ_RING_SIZE];
static double tx_iq_ring_q[TX_IQ_RING_SIZE];
static volatile int tx_iq_wr = 0; // written by poll thread
static volatile int tx_iq_rd = 0; // read by audio thread

#define TX_IQ_TIMEOUT_MS 500
static volatile unsigned long tx_iq_last_time_ms = 0;
static volatile int hpsdr_tx_data_active = 0;

// Previous sample for the 2× interpolation filter
static double tx_up_prev_i = 0.0;
static double tx_up_prev_q = 0.0;

static unsigned long millis_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned long)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void flush_tx_ring(void) {
  tx_iq_wr = 0;
  tx_iq_rd = 0;
  tx_up_prev_i = 0.0;
  tx_up_prev_q = 0.0;
}

// Write one 48 kHz sample pair into the ring as two 96 kHz samples
// using linear interpolation (simple half-band upsample).
static void tx_iq_push_48k(double i_val, double q_val) {
  double mid_i = 0.5 * (tx_up_prev_i + i_val);
  double mid_q = 0.5 * (tx_up_prev_q + q_val);
  int wr = tx_iq_wr;
  int rd = tx_iq_rd;

  // Overflow guard — drop if ring nearly full
  if ((wr - rd) >= (TX_IQ_RING_SIZE - 4))
    return;

  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = mid_i;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = mid_q;
  wr++;
  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = i_val;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = q_val;
  wr++;
  tx_iq_wr = wr;

  tx_up_prev_i = i_val;
  tx_up_prev_q = q_val;
  tx_iq_last_time_ms = millis_now();
}

int hpsdr_tx_iq_active(void) {
  if (!client_active)
    return 0;
  // Check that we received TX IQ data recently
  unsigned long now = millis_now();
  if (now - tx_iq_last_time_ms > TX_IQ_TIMEOUT_MS)
    return 0;
  // And that there is actually data in the ring
  return ((tx_iq_wr - tx_iq_rd) > 0);
}

int hpsdr_get_tx_iq(double *out_i, double *out_q, int max_samples) {
  int rd = tx_iq_rd;
  int avail = tx_iq_wr - rd;
  if (avail < 0)
    avail = 0;
  int n = (avail < max_samples) ? avail : max_samples;

  for (int k = 0; k < n; k++) {
    out_i[k] = tx_iq_ring_i[(rd + k) & TX_IQ_RING_MASK];
    out_q[k] = tx_iq_ring_q[(rd + k) & TX_IQ_RING_MASK];
  }
  tx_iq_rd = rd + n;
  return n;
}

// Called continuously with 96kHz samples from the RX processing chain.
// Applies a 24kHz half-band LPF to prevent aliasing, then decimates 2:1.
void hpsdr_send_iq(double *i_samples, double *q_samples, int n) {
  if (!client_active || hpsdr_sock < 0)
    return;

  for (int k = 0; k < n; k += 2) {
    double filt_i, filt_q;

    // Simple 3-tap FIR half-band filter: 0.25*z^-1 + 0.5*z^0 + 0.25*z^1
    // Cuts off accurately at Fs/4 (24kHz)
    if (k == 0) {
      filt_i = 0.25 * last_i + 0.5 * i_samples[0] + 0.25 * i_samples[1];
      filt_q = 0.25 * last_q + 0.5 * q_samples[0] + 0.25 * q_samples[1];
    } else if (k + 1 < n) {
      filt_i = 0.25 * i_samples[k - 1] + 0.5 * i_samples[k] + 0.25 * i_samples[k + 1];
      filt_q = 0.25 * q_samples[k - 1] + 0.5 * q_samples[k] + 0.25 * q_samples[k + 1];
    } else {
      // Edge case handling if 'n' isn't even, though it usually is 1024
      filt_i = i_samples[k];
      filt_q = q_samples[k];
    }

    iq_buf_i[iq_buf_count] = filt_i * hpsdr_iq_gain;
    iq_buf_q[iq_buf_count] = filt_q * hpsdr_iq_gain;
    iq_buf_count++;

    if (iq_buf_count >= SAMPLES_PER_PACKET) {
      build_and_send_packet();
      iq_buf_count = 0;
    }
  }
  // Save final samples for the next block's filter calculation
  if (n > 0) {
    last_i = i_samples[n - 1];
    last_q = q_samples[n - 1];
  }
}

// =============================================================================
// HPSDR PROTOCOL 1 IMPLEMENTATION
// =============================================================================

static volatile int remote_mox = 0;
static volatile unsigned long ep2_last_time_ms = 0;
#define EP2_WATCHDOG_MS 3500 // only for crash/disconnect

static int hpsdr_classify(const uint8_t *buf, int len) {
  if (len < 4 || buf[0] != 0xEF || buf[1] != 0xFE)
    return PKT_UNKNOWN;
  switch (buf[2]) {
  case 0x02:
    return PKT_DISCOVERY;
  case 0x04:
    return (buf[3] & 0x01) ? PKT_START : PKT_STOP;
  case 0x01:
    if (len >= HPSDR_PKT_SIZE)
      return PKT_EP2;
    return PKT_UNKNOWN;
  default:
    return PKT_UNKNOWN;
  }
}

static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use) {
  memset(reply, 0, HPSDR_DISCOVERY_REPLY);
  reply[0] = 0xEF;
  reply[1] = 0xFE;
  reply[2] = 0x02;
  reply[3] = in_use ? 0x02 : 0x00;
  reply[4] = 0x00; // MAC
  reply[5] = 0x1C;
  reply[6] = 0xC0;
  reply[7] = 0xA2;
  reply[8] = 0x22;
  reply[9] = 0x5B;
  reply[10] = 0x06; // board type (Hermes)
  reply[11] = 0x25; // protocol version
  reply[19] = 0x01; // number of receivers = 1
}

static void build_and_send_packet(void) {
  uint8_t pkt[HPSDR_PKT_SIZE];
  memset(pkt, 0, sizeof(pkt));

  // EP6 Header
  pkt[0] = 0xEF;
  pkt[1] = 0xFE;
  pkt[2] = 0x01;
  pkt[3] = 0x06;
  pkt[4] = (tx_seq >> 24) & 0xFF;
  pkt[5] = (tx_seq >> 16) & 0xFF;
  pkt[6] = (tx_seq >> 8) & 0xFF;
  pkt[7] = tx_seq & 0xFF;

  uint32_t seq_for_cc = tx_seq++;

  // Two 512-byte frames
  for (int frame = 0; frame < 2; frame++) {
    uint8_t *fp = pkt + 8 + frame * 512;

    // Sync bytes
    fp[0] = 0x7F;
    fp[1] = 0x7F;
    fp[2] = 0x7F;

    // C&C control bytes (cycle through C0 addresses 0 and 1)
    int cc_addr = (seq_for_cc * 2 + frame) % 2;
    fp[3] = (cc_addr << 1) | (in_tx ? 1 : 0);

    if (cc_addr == 0) {
      fp[4] = (freq_hdr >> 24) & 0xFF;
      fp[5] = (freq_hdr >> 16) & 0xFF;
      fp[6] = (freq_hdr >> 8) & 0xFF;
      fp[7] = freq_hdr & 0xFF;
    } else if (cc_addr == 1) {
      fp[4] = 0x00; // 48 kHz, no ADC overflow
      fp[5] = 0x00;
      fp[6] = 0x00;
      fp[7] = 0x00;
    }

    // 63 IQ samples per frame
    for (int s = 0; s < 63; s++) {
      int idx = frame * 63 + s;
      uint8_t *sp = fp + 8 + s * 8;

      int32_t i_val = (int32_t)(iq_buf_i[idx] * 559240.0);
      if (i_val > 8388607)
        i_val = 8388607;
      if (i_val < -8388608)
        i_val = -8388608;

      int32_t q_val = (int32_t)(iq_buf_q[idx] * 559240.0);
      if (q_val > 8388607)
        q_val = 8388607;
      if (q_val < -8388608)
        q_val = -8388608;

      // Pack I
      sp[0] = (i_val >> 16) & 0xFF;
      sp[1] = (i_val >> 8) & 0xFF;
      sp[2] = i_val & 0xFF;

      // Pack Q
      sp[3] = (q_val >> 16) & 0xFF;
      sp[4] = (q_val >> 8) & 0xFF;
      sp[5] = q_val & 0xFF;

      // Mic sample (unused)
      sp[6] = 0;
      sp[7] = 0;
    }
  }

  if (client_active) {
    sendto(hpsdr_sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&stream_dest, sizeof(stream_dest));
  }
}

static int hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result) {
  if (!buf || len < HPSDR_PKT_SIZE)
    return 0;
  result->mox = 0;
  result->freq = 0;
  result->n_samples = 0;

  const uint8_t *ptr = buf + 8; // skip Metis header
  for (int frame = 0; frame < 2; frame++) {
    if (ptr[0] != 0x7F || ptr[1] != 0x7F || ptr[2] != 0x7F) {
      ptr += 512;
      continue;
    }
    uint8_t c0 = ptr[3];
    uint8_t addr = (c0 >> 1) & 0x7F; // Juan's change
    int mox = c0 & 0x01;
    result->mox |= mox; // TX if *either* frame asserts MOX

    // Only update freq from the TX C&C address
    if (addr == 0x02) {
      result->freq = ((uint32_t)ptr[4] << 24) | ((uint32_t)ptr[5] << 16) |
                     ((uint32_t)ptr[6] << 8) | ((uint32_t)ptr[7]);
    }

    ptr += 8; // skip sync + C&C header
    for (int j = 0; j < 63 && result->n_samples < SAMPLES_PER_PKT; j++) {
      int16_t is = (int16_t)(((uint16_t)ptr[4] << 8) | (uint16_t)ptr[5]);
      int16_t qs = (int16_t)(((uint16_t)ptr[6] << 8) | (uint16_t)ptr[7]);
      result->iq[result->n_samples * 2 + 0] =
          (float)is / 32768.0f * hpsdr_tx_gain; // gain is added here
      result->iq[result->n_samples * 2 + 1] = (float)qs / 32768.0f * hpsdr_tx_gain;
      ptr += 8;
      result->n_samples++;
    }
  }
  return result->n_samples;
}

static void reset_all_tx_state(void) {
  flush_tx_ring();
  hpsdr_tx_data_active = 0;
  remote_mox = 0;
  ep2_last_time_ms = millis_now();
}

static void handle_command(uint8_t *buf, int len, struct sockaddr_in *sender) {
  int type = hpsdr_classify(buf, len);
  switch (type) {
  case PKT_DISCOVERY: {
    uint8_t reply[HPSDR_DISCOVERY_REPLY];
    int same = (sender->sin_addr.s_addr == stream_dest.sin_addr.s_addr);
    hpsdr_build_discovery_reply(reply, client_active && !same);
    sendto(hpsdr_sock, reply, sizeof(reply), 0, (struct sockaddr *)sender, sizeof(*sender));
    break;
  }
  case PKT_START:
    stream_dest = *sender;
    tx_seq = 0;
    iq_buf_count = 0;
    reset_all_tx_state();
    client_active = 1;
    printf("hpsdr: streaming STARTED to %s:%d\n", inet_ntoa(stream_dest.sin_addr),
           ntohs(stream_dest.sin_port));
    break;
  case PKT_STOP:
    client_active = 0;
    printf("hpsdr: streaming STOPPED\n");
    if (hpsdr_tx_data_active) {
      hpsdr_tx_data_active = 0;
      if (tr_pending != 2) {
        tr_pending = 2;
        g_idle_add(hpsdr_tr_idle, NULL);
      }
    }
    remote_mox = 0;
    break;
  case PKT_EP2: {
    if (!client_active)
      break;
    ep2_last_time_ms = millis_now();
    hpsdr_ep2_result_t r;
    hpsdr_unpack_ep2(buf, len, &r);

    // --- Frequency ---
    if (r.freq > 0 && r.freq != (uint32_t)freq_hdr) {
      printf("hpsdr: remote set freq %d Hz\n", r.freq);
      char cmd[50];
      sprintf(cmd, "freq %d", r.freq);
      remote_execute(cmd);
    }

    // --- MOX state machine (debounced, coalesced) ---
    {
      static int mox_count = 0;
      static int mox_pending_state = 0;

      if (r.mox != remote_mox) {
        // MOX differs from current state — count consecutive matches
        if (r.mox != mox_pending_state) {
          // Direction changed again — restart counter
          mox_pending_state = r.mox;
          mox_count = 1;
        } else {
          mox_count++;
        }

        if (mox_count >= 4) {
          // Stable for ~4 packets (~10 ms) — commit the transition
          remote_mox = mox_pending_state;
          mox_count = 0;
          if (remote_mox) {
            hpsdr_tx_data_active = 1;
            printf("hpsdr: MOX ON (debounced)\n");
            if (tr_pending != 1) {
              tr_pending = 1;
              g_idle_add(hpsdr_tr_idle, NULL);
            }
          } else {
            hpsdr_tx_data_active = 0;
            flush_tx_ring();
            printf("hpsdr: MOX OFF (debounced)\n");
            if (tr_pending != 2) {
              tr_pending = 2;
              g_idle_add(hpsdr_tr_idle, NULL);
            }
          }
        }
      } else {
        // MOX matches current state — reset debounce
        mox_count = 0;
        mox_pending_state = remote_mox;
      }
    }

    // --- TX IQ data (only when MOX is active) ---
    if (r.mox && r.n_samples > 0) {
      for (int k = 0; k < r.n_samples; k++)
        tx_iq_push_48k((double)r.iq[k * 2], (double)r.iq[k * 2 + 1]);
    }
    break;
  }
  }
}

// =============================================================================
// STATE TRANSLATION
// =============================================================================

// =============================================================================
// INITIALIZATION, CONTROL & SHUTDOWN
// =============================================================================

// T/R idle callback
// 0 = nothing pending, 1 = tx_on pending, 2 = tx_off pending
static gboolean hpsdr_tr_idle(gpointer data) {
  (void)data;
  int action = tr_pending;
  tr_pending = 0;

  if (action == 1 && !in_tx) {
    printf("hpsdr_tr_idle: switching to TX\n");
    tx_on(TX_SOFT);
  } else if (action == 2) {
    printf("hpsdr_tr_idle: switching to RX (in_tx=%d)\n", in_tx);
    tx_off();
  }
  return G_SOURCE_REMOVE;
}

static gboolean hpsdr_watchdog(gpointer data) {
  (void)data;
  if (!running)
    return G_SOURCE_REMOVE;
  if (in_tx && (millis_now() - ep2_last_time_ms > EP2_WATCHDOG_MS)) {
    printf("hpsdr watchdog: no EP2 for >%dms — forcing RX\n", EP2_WATCHDOG_MS);
    reset_all_tx_state();
    tx_off();
  }
  return G_SOURCE_CONTINUE;
}

static void *hpsdr_poll_thread(void *arg) {
  (void)arg;
  uint8_t buf[2048];
  struct sockaddr_in sender;
  socklen_t sender_len;

  while (running) {
    sender_len = sizeof(sender);
    int n = recvfrom(hpsdr_sock, buf, sizeof(buf), 0, (struct sockaddr *)&sender, &sender_len);
    if (n > 0) {
      handle_command(buf, n, &sender);
    }
  }
  return NULL;
}

int hpsdr_init(void) {
  hpsdr_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (hpsdr_sock < 0)
    return -1;

  int optval = 1;
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(HPSDR_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(hpsdr_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(hpsdr_sock);
    hpsdr_sock = -1;
    return -1;
  }

  struct timeval tv = {.tv_sec = 0, .tv_usec = 200000};
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  running = 1;
  return 0;
}

void hpsdr_stop(void) {
  running = 0;
  client_active = 0;
  if (hpsdr_sock >= 0) {
    close(hpsdr_sock);
    hpsdr_sock = -1;
  }
}

int hpsdr_is_connected(void) { return client_active; }

void hpsdr_poll(void) {
  static int started = 0;
  if (!started && running) {
    pthread_create(&poll_thread, NULL, hpsdr_poll_thread, NULL);
    g_timeout_add(250, hpsdr_watchdog, NULL); // fire every 250 ms on GTK thread
    started = 1;
  }
}
