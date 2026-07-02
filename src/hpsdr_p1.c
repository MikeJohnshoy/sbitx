// hpsdr_p1.c — HPSDR Protocol 1 interface for sBitx
// Now extended to provide 24bit I&Q via USB connection
// to support a broader range of external SDR applications.
//
// Provides the interface between the sBitx and an external SDR application
// using HPSDR Protocol 1, emulating a HermesLite radio.  With no SDR app present, 
// the module does nothing but listen on the UDP socket — completely invisible to
// normal sBitx operation.
//
// Data flow:
//   RX (sBitx → SDR app):  audio thread → hpsdr_send_iq() → [96k→48k decimation]
//                           → iq_buf → build_and_send_packet() → UDP/EP6 → SDR app
//   TX (SDR app → sBitx):  UDP/EP2 → hpsdr_unpack_ep2() → handle_command() → tx_upsample_and_push()
//                           → [48k→96k upsampling] → tx_iq_ring → hpsdr_get_tx_iq() → audio thread
//
// Major sections:
//  1. RX Signal Processing:  96k→48k half-band decimation, EP6 frame staging
//  2. TX Signal Processing;  48k→96k polyphase upsampling, ring buffer, public API
//  3. HPSDR Protocol 1:      packet I/O: classify, pack/unpack EP2/EP6, session control
//  4. sBitx State Integration:  translate HPSDR state into sBitx core calls (freq, T/R)
//  5. Initialization, Control & Shutdown:  socket setup, poll thread, watchdog
//
// There is support for data moving in both directions but the focus has been on receive functions.
//
// Inspired by Dave N1AI and Juan WP3DN
// Mike KB2ML

// System
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>

// Networking
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

// UI/Framework
#include <gtk/gtk.h>

// Local
#include "hpsdr_p1.h"

// -----------------------------------------------------------------------------
// Forward Declarations
// -----------------------------------------------------------------------------

// Shared utility
static unsigned long millis_now(void);

// Section 1: RX Signal Processing
static void rx_filter_and_decimate(double i0, double i1, double q0, double q1);
// Public: hpsdr_send_iq (defined in .h)

// Section 2: TX Signal Processing
static void flush_tx_ring(void);
static void tx_upsample_and_push(double i_val, double q_val);
// Public: hpsdr_tx_iq_active, hpsdr_get_tx_iq (defined in .h)

// Section 3: HPSDR Protocol 1
static int  hpsdr_classify(const uint8_t *buf, int len);
static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use);
static int  hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result);
static void build_and_send_packet(void);
static void reset_all_tx_state(void);
static void handle_command(uint8_t *buf, int len, struct sockaddr_in *sender);

// Section 4: sBitx State Integration
static void     apply_freq_from_ep2(uint32_t freq);
static void     apply_mox_from_ep2(int mox);
static gboolean hpsdr_tr_idle(gpointer data);

// Section 5: Initialization, Control & Shutdown
static gboolean hpsdr_watchdog(gpointer data);
static void    *hpsdr_poll_thread(void *arg);
// Public: hpsdr_init, hpsdr_stop, hpsdr_is_connected, hpsdr_poll (defined in .h)

// Section 6: USB
void uac_push_iq(double i_val, double q_val);
int  uac_is_active(void);

// -----------------------------------------------------------------------------
// Compile-time constants (protocol-independent)
// -----------------------------------------------------------------------------
#define TX_SOFT 2   // trigger code for software-initiated TX (passed to tx_on())

// -----------------------------------------------------------------------------
// Externs — sBitx core symbols this module drives
// -----------------------------------------------------------------------------
extern void remote_execute(char *command);
extern int  freq_hdr;
extern int  in_tx;
extern void tx_on(int trigger);
extern void tx_off(void);
extern void cmd_exec(char *cmd);

// -----------------------------------------------------------------------------
// Shared statics — variables accessed by two or more sections
//
//   Networking / session:
//     hpsdr_sock         — UDP socket fd; -1 when not open
//     stream_dest        — address/port of the currently connected SDR client
//     client_active      — 1 while a client session (START…STOP) is open
//     running            — 1 while the poll thread should keep looping
//
//   Protocol / DSP shared state:
//     hpsdr_sample_rate  — rate negotiated in EP2 addr 0x00 (48k or 96k);
//                          read by Section 1 (RX path) and written by Section 3 (EP2 decode)
//     remote_mox         — last committed (debounced) MOX state from SDR app;
//                          written by Section 4, read/reset by Section 3
//     tr_pending         — pending T/R action for GTK idle: 0=none 1=TX 2=RX;
//                          written by Sections 3 and 4, consumed by Section 4 idle callback
//     ep2_last_time_ms   — timestamp of last EP2 packet;
//                          written by Section 3, read by Section 5 watchdog
//     hpsdr_tx_data_active — 1 while remote TX IQ is actively flowing;
//                          written by Sections 3 and 4, read by Sections 2 and 3
// -----------------------------------------------------------------------------
static int                hpsdr_sock            = -1;
static struct sockaddr_in stream_dest;
static volatile int       client_active         = 0;
static volatile int       running               = 0;
static int                hpsdr_sample_rate     = 48000;

static volatile int           remote_mox           = 0;
static volatile int           tr_pending           = 0;
static volatile unsigned long ep2_last_time_ms     = 0;
static volatile int           hpsdr_tx_data_active = 0;

// Last non-zero RX1 (DDC0) frequency seen in EP2 addr 0x02
// persisted because the C&C round-robin only delivers this slot once every
// 19 frames — it is zero in the other 18.  This is the operator's selected
// signal frequency in both SDR Console (shown as "RX 1") and SPARK SDR
// (always at spectrum center).
static uint32_t last_rx_freq = 0;
// Last non-zero TX VFO frequency seen in EP2 addr 0x01.
// In SPARK SDR this is the selected signal frequency (spectrum center).
static uint32_t last_tx_freq = 0;

// -----------------------------------------------------------------------------
// Shared utility
// -----------------------------------------------------------------------------

// Return a monotonic millisecond timestamp. Used for timeouts and watchdogs
// across all sections.
static unsigned long millis_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned long)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// =============================================================================
// SECTION 1 — RX SIGNAL PROCESSING
// =============================================================================
// accept 96 kHz dual-channel IQ from the sBitx audio thread,
// optionally decimate 2:1 to 48 kHz using a half-band FIR, stage the result
// into iq_buf, and trigger an EP6 packet to the SDR app when the buffer is full.
//
// Data flow:
//   sBitx audio thread
//     → hpsdr_send_iq()              [entry point; selects 48k or 96k path]
//       → rx_filter_and_decimate()   [96k→48k half-band LPF + 2:1 decimation]
//         → iq_buf_i / iq_buf_q      [126-sample staging buffer]
//           → build_and_send_packet()  [triggered when buffer is full → EP6 UDP]
//
// Thread: called exclusively from the sBitx audio thread.

// 126-sample staging buffer — filled by the RX path, drained by build_and_send_packet()
static double iq_buf_i[SAMPLES_PER_PKT];
static double iq_buf_q[SAMPLES_PER_PKT];
static int    iq_buf_count = 0;

// Per-sample gain applied to outbound RX IQ before packing into EP6
static double hpsdr_iq_gain = 1.0;

// 31-tap half-band FIR coefficients (Fs=96k, cutoff=24k).
// Every other tap is 0 except the center tap (index 15 = 0.5).
// Declared static const so the compiler can fold zeros and exploit symmetry.
static const double hb_coeffs[31] = {
    -0.00055,  0.0,  0.00165,  0.0, -0.00411,  0.0,  0.00877,  0.0,
    -0.01736,  0.0,  0.03433,  0.0, -0.07612,  0.0,  0.30338,  0.5,
     0.30338,  0.0, -0.07612,  0.0,  0.03433,  0.0, -0.01736,  0.0,
     0.00877,  0.0, -0.00411,  0.0,  0.00165,  0.0, -0.00055
};

// Circular history buffer for the RX FIR (sized to power-of-two for masking)
static double rx_hist_i[32];
static double rx_hist_q[32];
static int    rx_hist_ptr = 0;

// Apply the 24 kHz half-band LPF to one input sample pair then decimate 2:1.
// i0/q0 is the older sample, i1/q1 is the newer; one output sample is produced.
// Uses FIR symmetry to reduce the 31-tap convolution to 9 multiply-adds.
static void rx_filter_and_decimate(double i0, double i1, double q0, double q1) {
  // Push newest sample pair into the circular history (newer sample first)
  rx_hist_ptr = (rx_hist_ptr - 1) & 31;
  rx_hist_i[rx_hist_ptr] = i1;
  rx_hist_q[rx_hist_ptr] = q1;

  rx_hist_ptr = (rx_hist_ptr - 1) & 31;
  rx_hist_i[rx_hist_ptr] = i0;
  rx_hist_q[rx_hist_ptr] = q0;

  // Center tap (index 15) is 0.5 — apply directly
  int    p      = rx_hist_ptr;
  double filt_i = rx_hist_i[(p + 15) & 31] * 0.5;
  double filt_q = rx_hist_q[(p + 15) & 31] * 0.5;

  // Exploit half-band symmetry: sum symmetric non-zero tap pairs (indices 0,2,...,14)
  // This reduces multiplications from 31 to 9.
  for (int j = 0; j < 15; j += 2) {
    double c       = hb_coeffs[j];
    int    idx_low = (p + j)      & 31;
    int    idx_hi  = (p + 30 - j) & 31;
    filt_i += (rx_hist_i[idx_low] + rx_hist_i[idx_hi]) * c;
    filt_q += (rx_hist_q[idx_low] + rx_hist_q[idx_hi]) * c;
  }

  // Stage the decimated sample; flush to EP6 when the buffer is full
  double out_i = filt_i * hpsdr_iq_gain;
  double out_q = filt_q * hpsdr_iq_gain;
  iq_buf_i[iq_buf_count] = out_i;
  iq_buf_q[iq_buf_count] = out_q;
  uac_push_iq(out_i, out_q);   // ← correct values, before increment
  iq_buf_count++;
  if (iq_buf_count >= SAMPLES_PER_PKT) {
    build_and_send_packet();
    iq_buf_count = 0;
  }
}

// Entry point called by the sBitx audio thread with 96 kHz IQ samples.
// Selects the 48 kHz decimation path or the 96 kHz pass-through based on
// the sample rate negotiated with the SDR app.
void hpsdr_send_iq(double *i_samples, double *q_samples, int n) {
  int hpsdr_live = client_active && (hpsdr_sock >= 0);
  int uac_live   = uac_is_active();
  if (!hpsdr_live && !uac_live) return;

  if (hpsdr_sample_rate == 48000) {
    // 2:1 decimation path: feed sample pairs into the half-band filter
    for (int k = 0; k < n - 1; k += 2) {
      rx_filter_and_decimate(i_samples[k], i_samples[k + 1],
                             q_samples[k], q_samples[k + 1]);
    }
  } else {
    // 96k, 192k, or 384k: pass through at native sBitx rate
    for (int k = 0; k < n; k++) {
        iq_buf_i[iq_buf_count] = i_samples[k] * hpsdr_iq_gain;
        iq_buf_q[iq_buf_count] = q_samples[k] * hpsdr_iq_gain;
        iq_buf_count++;
        if (iq_buf_count >= SAMPLES_PER_PKT) {
            build_and_send_packet();
            iq_buf_count = 0;
        }
    }
  }
}

// =============================================================================
// SECTION 2 — TX SIGNAL PROCESSING
// =============================================================================
// accept 48 kHz TX IQ from the SDR app (via hpsdr_unpack_ep2),
// upsample 2:1 to 96 kHz using a 6-tap polyphase FIR, and store the result in
// a lock-free ring buffer for consumption by the sBitx audio thread.
//
// Data flow:
//   SDR app (UDP/EP2)
//     → hpsdr_unpack_ep2()       [Section 3 — extracts raw 48k IQ samples]
//       → tx_upsample_and_push() [48k→96k polyphase upsampling]
//         → tx_iq_ring_i/q       [lock-free ring buffer, ~85 ms at 96k]
//           → hpsdr_get_tx_iq()  [consumed by sBitx audio thread]
//
// Thread safety: tx_iq_wr is written only by the poll thread; tx_iq_rd is
// written only by the audio thread. Both are _Atomic uint32_t — sufficient for a
// single-producer/single-consumer ring on a cache-coherent architecture.

// Lock-free ring buffer (size must be a power of two)
#define TX_IQ_RING_SIZE 8192
#define TX_IQ_RING_MASK (TX_IQ_RING_SIZE - 1)
static double       tx_iq_ring_i[TX_IQ_RING_SIZE];
static double       tx_iq_ring_q[TX_IQ_RING_SIZE];
static _Atomic uint32_t tx_iq_wr = 0;  // written by poll thread
static _Atomic uint32_t tx_iq_rd = 0;  // written by audio thread

// Declare the TX IQ stream stale if no new data arrives within this window
#define TX_IQ_TIMEOUT_MS 500
static volatile unsigned long tx_iq_last_time_ms = 0;

// Per-sample gain applied to inbound TX IQ during EP2 unpacking (Section 3)
static double hpsdr_tx_gain = 1.0;

// Delay lines for the 6-tap polyphase upsampling FIR
static double tx_hist_i[6] = {0};
static double tx_hist_q[6] = {0};

// Reset the TX ring buffer and clear FIR history.
// Called on session start, MOX-off, and watchdog timeout to prevent stale
// TX samples from leaking into a new transmission.
static void flush_tx_ring(void) {
  atomic_store_explicit(&tx_iq_wr, 0, memory_order_seq_cst);
  atomic_store_explicit(&tx_iq_rd, 0, memory_order_seq_cst);
  memset(tx_hist_i, 0, sizeof(tx_hist_i));
  memset(tx_hist_q, 0, sizeof(tx_hist_q));
}

// Upsample one 48 kHz IQ sample pair to two 96 kHz samples and push both
// into the TX ring buffer.
//
// A 6-tap polyphase half-band FIR generates the interpolated midpoint (Phase 1).
// The aligned original sample (Phase 0) is taken from the center of the delay
// line to maintain phase coherence with the midpoint.
static void tx_upsample_and_push(double i_val, double q_val) {
  // Shift the delay line and insert the new sample
  for (int i = 5; i > 0; i--) {
    tx_hist_i[i] = tx_hist_i[i - 1];
    tx_hist_q[i] = tx_hist_q[i - 1];
  }
  tx_hist_i[0] = i_val;
  tx_hist_q[0] = q_val;

  // 6-tap polyphase coefficients for the interpolated midpoint (Phase 1).
  // (replaced simple linear interpolation)
  static const double taps[6] = {0.0121, -0.0551, 0.2930,
                                  0.2930, -0.0551, 0.0121};

  // Phase 1: FIR-filtered interpolated midpoint
  double mid_i = 0, mid_q = 0;
  for (int i = 0; i < 6; i++) {
    mid_i += tx_hist_i[i] * taps[i];
    mid_q += tx_hist_q[i] * taps[i];
  }

  // Phase 0: aligned original sample (center of delay line)
  double out_i = tx_hist_i[2];
  double out_q = tx_hist_q[2];

   // Drop both samples if the ring is nearly full (overflow guard)
  uint32_t wr = atomic_load_explicit(&tx_iq_wr, memory_order_relaxed);
  uint32_t rd = atomic_load_explicit(&tx_iq_rd, memory_order_acquire);
  if ((uint32_t)(wr - rd) >= (TX_IQ_RING_SIZE - 4))
    return;

  // Write midpoint first, then aligned original
  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = mid_i;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = mid_q;
  wr++;

  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = out_i;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = out_q;
  wr++;

  // release: guarantees ring data is visible before the new index is
  atomic_store_explicit(&tx_iq_wr, wr, memory_order_release);
  tx_iq_last_time_ms = millis_now();
}

// Returns 1 if the SDR app is actively supplying TX IQ data:
//   - a client session must be open
//   - a TX IQ packet must have arrived within the last TX_IQ_TIMEOUT_MS
//   - the ring buffer must contain at least one sample pair
int hpsdr_tx_iq_active(void) {
  if (!client_active)
    return 0;
  if (millis_now() - tx_iq_last_time_ms > TX_IQ_TIMEOUT_MS)
    return 0;
  uint32_t wr = atomic_load_explicit(&tx_iq_wr, memory_order_acquire);
  uint32_t rd = atomic_load_explicit(&tx_iq_rd, memory_order_relaxed);
  return ((uint32_t)(wr - rd) > 0);
}

int hpsdr_get_tx_iq(double *out_i, double *out_q, int max_samples) {
  uint32_t rd    = atomic_load_explicit(&tx_iq_rd, memory_order_relaxed);
  uint32_t wr    = atomic_load_explicit(&tx_iq_wr, memory_order_acquire);
  uint32_t avail = (uint32_t)(wr - rd);
  int n = ((int)avail < max_samples) ? (int)avail : max_samples;

  for (int k = 0; k < n; k++) {
    out_i[k] = tx_iq_ring_i[(rd + k) & TX_IQ_RING_MASK];
    out_q[k] = tx_iq_ring_q[(rd + k) & TX_IQ_RING_MASK];
  }
  atomic_store_explicit(&tx_iq_rd, rd + (uint32_t)n, memory_order_release);
  return n;
}

// =============================================================================
// SECTION 3 — HPSDR PROTOCOL 1
// =============================================================================
//  all packet-level I/O between this module and the SDR app.
// No sBitx hardware state is changed here; that is delegated to Section 4.
//
// Inbound  (SDR app → sBitx): PKT_DISCOVERY, PKT_START, PKT_STOP, PKT_EP2
// Outbound (sBitx → SDR app): EP6 RX IQ stream
//
// Function order within this section:
//   classify → inbound handlers (discovery, EP2 unpack) →
//   outbound builder (EP6) → session reset → top-level dispatcher

// EP6 sequence counter — incremented with every outbound packet
static uint32_t tx_seq = 0;

// Classify an inbound UDP packet by inspecting its header bytes.
// Returns one of the PKT_* constants defined in hpsdr_p1.h.
static int hpsdr_classify(const uint8_t *buf, int len) {
  if (len < 4 || buf[0] != 0xEF || buf[1] != 0xFE)
    return PKT_UNKNOWN;
  switch (buf[2]) {
    case 0x02: return PKT_DISCOVERY;
    case 0x04: return (buf[3] & 0x01) ? PKT_START : PKT_STOP;
    case 0x01: return (len >= HPSDR_PKT_SIZE) ? PKT_EP2 : PKT_UNKNOWN;
    default:   return PKT_UNKNOWN;
  }
}

// Build a 60-byte discovery reply identifying this device as a HermesLite.
// in_use is set when a session is already active with a different client,
// signalling to the requester that the radio is busy.
static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use) {
  memset(reply, 0, HPSDR_DISCOVERY_REPLY);  // HPSDR_DISCOVERY_REPLY currently defined as 60
  reply[0] = 0xEF;
  reply[1] = 0xFE;
  reply[2] = 0x02;
  reply[3] = in_use ? 0x02 : 0x00;

  // MAC address (fixed, HermesLite-style)
  reply[4] = 0x00;
  reply[5] = 0x1C; reply[6] = 0xC0; reply[7] = 0xA2;
  reply[8] = 0x22; reply[9] = 0x5B;

  reply[10] = 0x06; // Board type: Hermes-Lite
  reply[11] = 0x4A; // Firmware version 74 dec
  reply[19] = 0x01; // MetisVersion
  reply[20] = 0x01; // NumRxs = 1
}

// Unpack one 1032-byte EP2 packet (SDR app → sBitx).
// Extracts the MOX bit, RX/TX frequencies from the C&C round-robin, and up
// to 126 TX IQ sample pairs (16-bit big-endian, normalised to ±1.0).
// Returns the number of IQ sample pairs placed in result->iq[].
static int hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result) {
  if (!buf || len < HPSDR_PKT_SIZE) return 0;

  result->mox       = 0;
  result->freq      = 0;
  result->tx_freq   = 0;
  result->n_samples = 0;

  const uint8_t *ptr = buf + 8; // skip 8-byte Metis header

  for (int frame = 0; frame < 2; frame++) {
    // Validate USB sync bytes
    if (ptr[0] != 0x7F || ptr[1] != 0x7F || ptr[2] != 0x7F) {
      ptr += 512;
      continue;
    }

    uint8_t c0   = ptr[3];
    uint8_t addr = (c0 >> 1) & 0x7F; // C&C round-robin slot index
    int     mox  = c0 & 0x01;        // MOX/PTT bit (Section 8.3)

    // OR the MOX bit across both frames (Section 5.5)
    result->mox |= mox;

    // 19 standard slots decoded; HL2 extension slots listed but not used
    switch (addr) {
      case 0x00: { // General Settings: sample rate in C1 bits 1:0
        uint8_t rate_bits = ptr[4] & 0x03;
        if      (rate_bits == 0) hpsdr_sample_rate =  48000;
        else if (rate_bits == 1) hpsdr_sample_rate =  96000;
        else if (rate_bits == 2) hpsdr_sample_rate = 192000;
        else if (rate_bits == 3) hpsdr_sample_rate = 384000;
        break;
      }
      case 0x01: // TX VFO frequency
        result->tx_freq = ((uint32_t)ptr[4] << 24) | ((uint32_t)ptr[5] << 16) |
                          ((uint32_t)ptr[6] <<  8) |  (uint32_t)ptr[7];
        //printf("hpsdr EP2 addr 0x01 tx_freq = %u\n", result->tx_freq);
        break;
      case 0x02: // RX1 (DDC0) frequency
        result->freq    = ((uint32_t)ptr[4] << 24) | ((uint32_t)ptr[5] << 16) |
                          ((uint32_t)ptr[6] <<  8) |  (uint32_t)ptr[7];
        //printf("hpsdr EP2 addr 0x02 freq    = %u\n", result->freq);
        break;
      case 0x03: // RX2 (DDC1) frequency — not used
      case 0x0E: // ADC assignments & TX step attenuator — not used
      case 0x04: case 0x05: case 0x06: case 0x07: case 0x08: // DDC2-6 — not used
      case 0x09: // Drive level & Alex filters — stub (sBitx filter relays go here)
      case 0x0A: // Preamp & RX step attenuator — not used
      case 0x0B: // CW keyer speed & weight — not used
      case 0x0F: // CW enable & sidetone level — not used
      case 0x10: // CW hang delay & sidetone freq — not used
      case 0x11: // EER PWM settings — not used
      case 0x12: // BPF2 / transverter — not used
      case 0x17: // HL2 extension (C0 masked 0x2E) — not used
      case 0x3A: // HL2 extension (C0 masked 0x74) — not used
      default:
        break;
    }

    // Advance past the 8-byte sync+C&C header to the IQ payload
    ptr += 8;
    // Unpack 63 TX IQ sample pairs per USB frame (Section 8.4).
    // Each group is 8 bytes: [L audio 0-1][R audio 2-3][I 4-5][Q 6-7]
    for (int j = 0; j < 63 && result->n_samples < SAMPLES_PER_PKT; j++) {
      int16_t is = (int16_t)(((uint16_t)ptr[4] << 8) | (uint16_t)ptr[5]);
      int16_t qs = (int16_t)(((uint16_t)ptr[6] << 8) | (uint16_t)ptr[7]);

      result->iq[result->n_samples * 2 + 0] = (float)is / 32768.0f * hpsdr_tx_gain;
      result->iq[result->n_samples * 2 + 1] = (float)qs / 32768.0f * hpsdr_tx_gain;

      ptr += 8;
      result->n_samples++;
    }
  }
  return result->n_samples;
}

// Build and send one 1032-byte EP6 packet (sBitx → SDR app).
// Contains two USB frames of 63 IQ sample pairs (24-bit big-endian) drawn
// from iq_buf, plus C&C bytes reporting current sBitx state via a 5-slot
// round-robin (Section 4.3).
// Called from rx_filter_and_decimate() / hpsdr_send_iq() when iq_buf is full.
static void build_and_send_packet(void) {
  uint8_t pkt[HPSDR_PKT_SIZE];
  memset(pkt, 0, sizeof(pkt));

  // Metis header: EP6 (radio → host)
  pkt[0] = 0xEF; pkt[1] = 0xFE; pkt[2] = 0x01; pkt[3] = 0x06;
  pkt[4] = (tx_seq >> 24) & 0xFF;
  pkt[5] = (tx_seq >> 16) & 0xFF;
  pkt[6] = (tx_seq >>  8) & 0xFF;
  pkt[7] =  tx_seq        & 0xFF;

  uint32_t seq_for_cc = tx_seq++;

  for (int frame = 0; frame < 2; frame++) {
    uint8_t *fp = pkt + 8 + frame * 512;
    fp[0] = 0x7F; fp[1] = 0x7F; fp[2] = 0x7F; // USB sync

    // 5-slot C&C round-robin: advance one slot per frame sent
    int cc_addr = (seq_for_cc * 2 + frame) % 5;

    // C0: slot index in bits 7:3, current PTT/MOX state in bit 0
    fp[3] = (cc_addr << 3) | (in_tx ? 1 : 0);

    // C1–C4: slot payload (Section 4.3)
    switch (cc_addr) {
      case 0: // ADC overload flags & firmware version
        fp[4] = 0x00; // C1: ADC overload (0 = no clip)
        fp[5] = 0x00; // C2: digital inputs
        fp[6] = 0x4A; // C3: firmware version 74
        fp[7] = 0x00; // C4: reserved
        break;
      case 1: // Exciter power (C1-C2) & forward PA power (C3-C4)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
      case 2: // Reverse PA power (C1-C2) & PA voltage / User_ADC0 (C3-C4)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
      case 3: // PA current / User_ADC1 (C1-C2) & supply voltage (C3-C4)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
      case 4: // Additional ADC overload flags: ADC0 (C1), ADC1 (C2), ADC2 (C3)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
    }

    // 63 IQ sample pairs per frame, packed as 24-bit big-endian I then Q,
    // followed by 2 bytes of mic audio (unused, left as zero)
    for (int s = 0; s < 63; s++) {
      int     idx = frame * 63 + s;
      uint8_t *sp = fp + 8 + s * 8;

      int32_t i_val = (int32_t)(iq_buf_i[idx] * 559240.0);
      if (i_val >  8388607)  i_val =  8388607;
      if (i_val < -8388608)  i_val = -8388608;

      int32_t q_val = (int32_t)(iq_buf_q[idx] * 559240.0);
      if (q_val >  8388607)  q_val =  8388607;
      if (q_val < -8388608)  q_val = -8388608;

      sp[0] = (i_val >> 16) & 0xFF;
      sp[1] = (i_val >>  8) & 0xFF;
      sp[2] =  i_val        & 0xFF;
      sp[3] = (q_val >> 16) & 0xFF;
      sp[4] = (q_val >>  8) & 0xFF;
      sp[5] =  q_val        & 0xFF;
      sp[6] = 0; // mic high byte (unused)
      sp[7] = 0; // mic low byte  (unused)
    }
  }

  if (client_active) {
    sendto(hpsdr_sock, pkt, sizeof(pkt), 0,
           (struct sockaddr *)&stream_dest, sizeof(stream_dest));
  }
}

// Reset all TX-related state to a clean idle condition.
// This is the single chokepoint for TX teardown — ensures no stale state
// carries over between sessions or after a crash/disconnect.
// Called on PKT_START, PKT_STOP, and watchdog timeout.
static void reset_all_tx_state(void) {
  flush_tx_ring();
  hpsdr_tx_data_active = 0;
  remote_mox           = 0;
  ep2_last_time_ms     = millis_now();
}

// Top-level packet dispatcher: classify each inbound UDP packet and route it
// to the appropriate handler. EP2 packets are unpacked here, then handed off
// to Section 4 for sBitx state changes.
static void handle_command(uint8_t *buf, int len, struct sockaddr_in *sender) {
  switch (hpsdr_classify(buf, len)) {

  case PKT_DISCOVERY: {
    uint8_t reply[HPSDR_DISCOVERY_REPLY];
    // Report "in use" only if the active session belongs to a different host
    int same = (sender->sin_addr.s_addr == stream_dest.sin_addr.s_addr);
    hpsdr_build_discovery_reply(reply, client_active && !same);
    sendto(hpsdr_sock, reply, sizeof(reply), 0,
           (struct sockaddr *)sender, sizeof(*sender));
    break;
  }

  case PKT_START:
    stream_dest       = *sender;
    tx_seq            = 0;
    iq_buf_count      = 0;
    hpsdr_sample_rate = 48000; // reset to default; client will re-negotiate
    reset_all_tx_state();
    last_rx_freq = 0;
    last_tx_freq = 0;
    client_active = 1;
    printf("hpsdr: streaming STARTED to %s:%d\n",
           inet_ntoa(stream_dest.sin_addr), ntohs(stream_dest.sin_port));
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
    if (!client_active) break;
    ep2_last_time_ms = millis_now();

    hpsdr_ep2_result_t r;
    hpsdr_unpack_ep2(buf, len, &r);

    // Persist the RX1 frequency across the 19-slot round-robin gap.
    // addr 0x02 is zero in 18 of every 19 frames.
    if (r.freq) last_rx_freq = r.freq;
    if (r.tx_freq) last_tx_freq = r.tx_freq;

    // During RX, follow the spectrum LO continuously.
    // During TX (either remote MOX or physical key), skip — must not override
    // the TX VFO back to the RX LO on every EP2 packet.
    if (!remote_mox && !in_tx)
      apply_freq_from_ep2(last_rx_freq);
    apply_mox_from_ep2(r.mox);

    for (int k = 0; k < r.n_samples; k++)
      tx_upsample_and_push(r.iq[k * 2], r.iq[k * 2 + 1]);
    break;
  }

  }   // switch statement
}

// =============================================================================
// SECTION 4 — sBITX STATE INTEGRATION
// =============================================================================
// translate HPSDR protocol state into sBitx hardware/core
// actions. No packet I/O happens here; this section only drives the sBitx
// externals (remote_execute, tx_on, tx_off) in response to decoded EP2 data.
//
// Thread note: apply_freq_from_ep2() and apply_mox_from_ep2() run on the
// poll thread. The T/R switch calls (tx_on/tx_off) must run on the GTK main
// thread, so they are deferred via g_idle_add(hpsdr_tr_idle).
//
// Future additions: gain settings, band info, antenna selection, etc.

// Apply a decoded RX or TX VFO frequency to the sBitx, but only when it
// differs from the current sBitx tuned frequency.
static void apply_freq_from_ep2(uint32_t freq) {
  if (freq > 0 && freq != (uint32_t)freq_hdr) {
    //printf("hpsdr: remote set freq %u Hz\n", freq);
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "freq %u", freq);
    remote_execute(cmd);
  }
}

// Translate the EP2 MOX bit into a debounced T/R switch action.
// The new state must be stable for 4 consecutive EP2 packets (~10 ms)
// before the transition is committed, preventing glitches from a single
// spurious packet or brief contact bounce.
static void apply_mox_from_ep2(int mox) {
  static int mox_count         = 0;
  static int mox_pending_state = 0;

  if (mox != remote_mox) {
    // Incoming MOX differs from committed state — accumulate stable count
    if (mox != mox_pending_state) {
      // Direction changed before threshold — restart the counter
      mox_pending_state = mox;
      mox_count = 1;
    } else {
      mox_count++;
    }

    if (mox_count >= 4) {
      // Stable for ~4 packets (~10 ms) — commit the transition
      remote_mox = mox_pending_state;
      mox_count  = 0;
      if (remote_mox) {
        // Tune to operator's selected signal before the T/R switch fires
        //if (last_rx_freq) apply_freq_from_ep2(last_rx_freq);
        hpsdr_tx_data_active = 1;
        //printf("hpsdr: MOX ON (debounced)\n");
        if (tr_pending != 1) {
          tr_pending = 1;
          g_idle_add(hpsdr_tr_idle, NULL);
        }
      } else {
        hpsdr_tx_data_active = 0;
        flush_tx_ring();
        // Restore the RX spectrum centre VFO when returning to receive
        if (last_rx_freq) apply_freq_from_ep2(last_rx_freq);
        //printf("hpsdr: MOX OFF (debounced)\n");
        if (tr_pending != 2) {
          tr_pending = 2;
          g_idle_add(hpsdr_tr_idle, NULL);
        }
      }
    }
  } else {
    // MOX matches committed state — reset debounce counters
    mox_count         = 0;
    mox_pending_state = remote_mox;
  }
}

// GTK main-thread callback to execute a pending T/R switch.
// Deferred here because tx_on()/tx_off() must not be called from the poll thread.
// tr_pending values: 0 = nothing, 1 = go TX, 2 = go RX
static gboolean hpsdr_tr_idle(gpointer data) {
  (void)data;
  int action = tr_pending;
  tr_pending = 0;

  if (action == 1 && !in_tx) {
    // addr 0x01 = operator's selected signal ("RX 1" in SDR Console).
    // For SPARK SDR the selected signal is always the spectrum center so
    // last_tx_freq == last_rx_freq; fall back to last_rx_freq if addr 0x01
    // was never received.
    uint32_t tx_freq = last_tx_freq ? last_tx_freq : last_rx_freq;
    //printf("hpsdr_tr_idle: last_rx_freq=%u freq_hdr=%d\n", last_rx_freq, freq_hdr);
    if (tx_freq) {
      char cmd[50];
      snprintf(cmd, sizeof(cmd), "freq %u", tx_freq);
      cmd_exec(cmd);
      //printf("hpsdr_tr_idle: set TX freq to %u Hz\n", tx_freq);
    }
    //printf("hpsdr_tr_idle: switching to TX\n");
    tx_on(TX_SOFT);
  } else if (action == 2) {
    //printf("hpsdr_tr_idle: switching to RX (in_tx=%d)\n", in_tx);
    tx_off();
  }
  return G_SOURCE_REMOVE;
}

// =============================================================================
// SECTION 5 — INITIALIZATION, CONTROL & SHUTDOWN
// =============================================================================
// manage the lifecycle of the HPSDR interface — open/close the
// UDP socket, spawn the poll thread, run the EP2 watchdog, and expose the
// public control API used by sbitx.c.
//
// The poll thread runs hpsdr_poll_thread() which blocks on recvfrom() and
// dispatches every inbound packet through handle_command() (Section 3).
// The watchdog fires every 50 ms on the GTK main thread and forces a return
// to RX if no EP2 has been received for EP2_WATCHDOG_MS while in TX — covering
// crash, network drop, or any other unclean disconnect.

// Poll thread handle — used only within this section
static pthread_t poll_thread;
static int       poll_thread_started = 0;

// Force RX after this many ms of EP2 silence while in TX
#define EP2_WATCHDOG_MS 3500

static gboolean hpsdr_watchdog(gpointer data) {
  (void)data;
  static int last_in_tx = 0;

  if (!running)
    return G_SOURCE_REMOVE;

  // the 'tr_switch' for keying the sbitx while connected to a SDR app
  // using cmd_exec and tx_on
  // Detect physical key/PTT press: in_tx transitioned to 1 without us
  // initiating it via MOX.  Correct the frequency to the SDR app's selected
  // signal now that we're on the GTK main thread where cmd_exec is safe.
  if (client_active && in_tx && !last_in_tx && !remote_mox) {
    // physical key/PTT only — MOX path already set freq in hpsdr_tr_idle
    uint32_t tx_freq = last_tx_freq ? last_tx_freq : last_rx_freq;
    if (tx_freq) {
      char cmd[50];
      snprintf(cmd, sizeof(cmd), "freq %u", tx_freq);
      //printf("hpsdr watchdog: TX detected, last_rx_freq=%u freq_hdr=%d remote_mox=%d\n", last_rx_freq, freq_hdr, remote_mox);
      cmd_exec(cmd);
      //printf("hpsdr watchdog: PTT detected, corrected TX freq to %u Hz\n", tx_freq);
    }
  }
  last_in_tx = in_tx;

  // Existing watchdog: force RX if EP2 goes silent while in TX
  if (client_active && remote_mox && in_tx &&
      (millis_now() - ep2_last_time_ms > EP2_WATCHDOG_MS)) {
    //printf("hpsdr watchdog: no EP2 for >%dms — forcing RX\n", EP2_WATCHDOG_MS);
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
    int n = recvfrom(hpsdr_sock, buf, sizeof(buf), 0,
                     (struct sockaddr *)&sender, &sender_len);
    if (n > 0)
      handle_command(buf, n, &sender);
  }
  return NULL;
}

// Open the UDP socket bound to HPSDR_PORT and prepare it for use.
// Returns 0 on success, -1 on failure.
int hpsdr_init(void) {
  hpsdr_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (hpsdr_sock < 0) return -1;

  int optval = 1;
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(HPSDR_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(hpsdr_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(hpsdr_sock);
    hpsdr_sock = -1;
    return -1;
  }

  // 200 ms receive timeout so the poll thread can check `running` periodically
  struct timeval tv = {.tv_sec = 0, .tv_usec = 200000};
  setsockopt(hpsdr_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  running = 1;
  return 0;
}

// Close the socket and mark the interface as stopped.
// The poll thread will exit at its next recvfrom() timeout.
void hpsdr_stop(void) {
  running       = 0;
  client_active = 0;
  if (hpsdr_sock >= 0) {
    close(hpsdr_sock);
    hpsdr_sock = -1;
  }
  if (poll_thread_started) {
    pthread_join(poll_thread, NULL);   // blocks until poll thread exits cleanly
    poll_thread_started = 0;           // reset so hpsdr_init/poll can restart safely
  }
}

// Returns 1 if a client session is currently active.
int hpsdr_is_connected(void) {
  return client_active;
}

// Start the poll thread and watchdog timer on the first call after hpsdr_init().
// Safe to call repeatedly — the thread and timer are created only once.
void hpsdr_poll(void) {
  if (!poll_thread_started && running) {
    pthread_create(&poll_thread, NULL, hpsdr_poll_thread, NULL);
    g_timeout_add(50, hpsdr_watchdog, NULL);  // 50 ms timer
    poll_thread_started = 1;
  }
}

// =============================================================================
// SECTION 6 — USB AUDIO CLASS (UAC2) IQ STREAM
// =============================================================================
// Provides a USB Audio Class 2.0 gadget device named "sBitx" that streams
// 24-bit / 48 kHz stereo IQ (I = left, Q = right) to any SDR application
// that can consume a USB audio input (e.g. SDR#, HDSDR, GQRX, SDR Console).
//
// Architecture overview:
//   The Linux USB gadget framework is configured via the configfs API under
//   /sys/kernel/config/usb_gadget/.  A UAC2 function is bound with:
//     - bcdADC = 0x0200 (Audio Class 2.0)
//     - one AudioStreaming interface: 2-ch, 24-bit PCM, 48000 Hz
//     - device/product strings: "sBitx"
//   Once the gadget is bound to a UDC controller (detected automatically),
//   the host sees a standard USB audio capture device called "sBitx".
//
//   Sample delivery:
//     hpsdr_send_iq() (Section 1) already decimates to 48 kHz and stages
//     samples in iq_buf_i / iq_buf_q.  uac_push_iq() is called from the
//     same audio thread after each decimated sample pair is ready and writes
//     packed 24-bit PCM frames to the ALSA loopback that the UAC2 gadget
//     reads.  This keeps UAC2 in sync with the existing HPSDR path at zero
//     added DSP cost.
//
//   ALSA loopback bridge:
//     Linux's snd-aloop module creates a pair of back-to-back PCM devices.
//     The gadget's UAC2 function reads from one side; we write to the other
//     via a standard PCM write call.  This avoids any kernel-module custom
//     code and works on any Linux distro with snd-aloop loaded.
//
// Configfs gadget path layout (created by uac_gadget_init):
//   /sys/kernel/config/usb_gadget/sbitx_iq/
//     idVendor, idProduct, bcdUSB, bcdDevice
//     strings/0x409/manufacturer  = "sBitx"
//     strings/0x409/product       = "sBitx IQ"
//     strings/0x409/serialnumber  = "0000001"
//     configs/c.1/
//       strings/0x409/configuration = "Default"
//       bmAttributes, MaxPower
//       function symlink → functions/uac2.0/
//     functions/uac2.0/
//       c_srate  = 48000
//       c_ssize  = 3        (3 bytes = 24-bit)
//       c_chmask = 3        (2 channels: L=I, R=Q)
//       p_srate  = 48000    (playback side, unused but must be set)
//       p_ssize  = 3
//       p_chmask = 3
//
// Public API (declared at bottom of hpsdr_p1.h):
//   int  uac_init(void)     — configure gadget + open ALSA loopback PCM
//   void uac_push_iq(double i_val, double q_val)
//                           — deliver one 48 kHz sample pair (called per-sample
//                              from hpsdr_send_iq after decimation)
//   void uac_stop(void)     — tear down gadget and release PCM handle
//   int  uac_is_active(void)— returns 1 while a host is reading the stream
//
// Dependencies (must be present on the target system):
//   Kernel modules : dwc2 (or other device-mode UDC), libcomposite, snd-aloop
//   Userspace libs : libasound2-dev  (ALSA — for PCM write to loopback)
//   Kernel config  : CONFIG_USB_CONFIGFS_F_UAC2=y
//
// Build addition:
//   Add  -lasound  to the sBitx link flags.
//
// Thread safety:
//   uac_init / uac_stop run on the GTK main thread.
//   uac_push_iq runs on the sBitx audio thread (same as hpsdr_send_iq).
//   No locking is needed — the ALSA PCM handle is only written by one thread.

#include <alsa/asoundlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

// ---------------------------------------------------------------------------
// Compile-time configuration
// ---------------------------------------------------------------------------

// Root of the Linux USB gadget configfs hierarchy
#define UAC_GADGET_ROOT     "/sys/kernel/config/usb_gadget/sbitx_iq"

// ALSA loopback device that the UAC2 gadget reads.
// snd-aloop creates hw:Loopback,0,0 (capture) and hw:Loopback,0,1 (playback).
// We write to the playback side; the UAC2 gadget reads from the capture side.
// The card index may vary; uac_init() probes for it automatically.
#define UAC_LOOPBACK_BASE   "hw:Loopback"

// PCM parameters to match the UAC2 descriptor
#define UAC_RATE            48000
#define UAC_CHANNELS        2
#define UAC_SAMPLE_BITS     24          // bits per sample (in 32-bit container)
#define UAC_SAMPLE_BYTES    3           // packed on-wire bytes (24-bit PCM)
#define UAC_PERIOD_FRAMES   512         // ALSA period size in frames
#define UAC_PERIODS         4           // number of periods in the ring buffer

// One frame = UAC_CHANNELS * UAC_SAMPLE_BYTES bytes
#define UAC_FRAME_BYTES     (UAC_CHANNELS * UAC_SAMPLE_BYTES)

// Internal ring: hold up to UAC_PERIOD_FRAMES samples before each ALSA write
#define UAC_BUF_FRAMES      UAC_PERIOD_FRAMES
static uint8_t  uac_pcm_buf[UAC_BUF_FRAMES * UAC_FRAME_BYTES];
static int      uac_buf_pos  = 0;   // next frame slot to fill (in frames)

// ---------------------------------------------------------------------------
// Module-level state
// ---------------------------------------------------------------------------
static snd_pcm_t *uac_pcm_handle  = NULL;  // ALSA PCM write handle
static int        uac_gadget_up   = 0;     // 1 after configfs gadget is created
static volatile int uac_active    = 0;     // 1 while host is streaming

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Write a NUL-terminated string to a sysfs/configfs attribute file.
// Returns 0 on success, -1 on error (errno is preserved).
static int uac_write_attr(const char *path, const char *value) {
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return -1;
    ssize_t n = write(fd, value, strlen(value));
    close(fd);
    return (n == (ssize_t)strlen(value)) ? 0 : -1;
}

// Create a directory if it does not already exist.
// Mirrors `mkdir -p` for a single level.
static int uac_mkdir(const char *path) {
    if (mkdir(path, 0755) < 0 && errno != EEXIST) return -1;
    return 0;
}

// Create a symbolic link, tolerating EEXIST.
static int uac_symlink(const char *target, const char *link) {
    if (symlink(target, link) < 0 && errno != EEXIST) return -1;
    return 0;
}

// Probe for the first available ALSA loopback card by scanning /proc/asound.
// Sets card_idx to the card number and returns 0 on success, -1 if not found.
static int uac_find_loopback_card(int *card_idx) {
    for (int c = 0; c < 32; c++) {
        char path[64];
        snprintf(path, sizeof(path), "/proc/asound/card%d/id", c);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        char id[64] = {0};
        if (fgets(id, sizeof(id), f)) {
            // Strip trailing newline
            id[strcspn(id, "\n")] = '\0';
            if (strcmp(id, "Loopback") == 0) {
                *card_idx = c;
                fclose(f);
                return 0;
            }
        }
        fclose(f);
    }
    return -1;   // snd-aloop not loaded or no loopback card present
}

// Detect the first UDC (USB Device Controller) available on this system by
// listing /sys/class/udc/.  Copies the UDC name into buf (max len).
// Returns 0 on success, -1 if no UDC is found.
static int uac_find_udc(char *buf, size_t len) {
    DIR *d = opendir("/sys/class/udc");
    if (!d) return -1;
    struct dirent *de;
    while ((de = readdir(d))) {
        if (de->d_name[0] == '.') continue;
        snprintf(buf, len, "%s", de->d_name);
        closedir(d);
        return 0;
    }
    closedir(d);
    return -1;
}

// ---------------------------------------------------------------------------
// Gadget configuration via configfs
// ---------------------------------------------------------------------------

// Create and configure the UAC2 gadget under configfs.
// This is equivalent to the shell script in the design notes but implemented
// in C so that sBitx controls the gadget lifecycle directly.
//
// Idempotent: if the gadget already exists (leftover from a crash), this
// function detects the existing tree and skips redundant mkdir/write calls.
//
// Returns 0 on success, -1 on any configfs error.
static int uac_gadget_create(void) {
    char path[256];

    // --- Gadget root ---
    if (uac_mkdir(UAC_GADGET_ROOT) < 0) {
        fprintf(stderr, "uac: cannot create gadget root %s: %s\n",
                UAC_GADGET_ROOT, strerror(errno));
        return -1;
    }

    // USB IDs: use the HermesLite vendor/product pair to stay compatible with
    // SDR apps that enumerate by USB ID, while the product string distinguishes us.
    uac_write_attr(UAC_GADGET_ROOT "/idVendor",  "0x04B4");   // Cypress / generic
    uac_write_attr(UAC_GADGET_ROOT "/idProduct", "0x0008");   // generic audio
    uac_write_attr(UAC_GADGET_ROOT "/bcdUSB",    "0x0200");   // USB 2.0
    uac_write_attr(UAC_GADGET_ROOT "/bcdDevice", "0x0100");

    // --- String descriptors (English) ---
    snprintf(path, sizeof(path), "%s/strings/0x409", UAC_GADGET_ROOT);
    uac_mkdir(path);
    snprintf(path, sizeof(path), "%s/strings/0x409/manufacturer", UAC_GADGET_ROOT);
    uac_write_attr(path, "sBitx");
    snprintf(path, sizeof(path), "%s/strings/0x409/product", UAC_GADGET_ROOT);
    uac_write_attr(path, "sBitx IQ");
    snprintf(path, sizeof(path), "%s/strings/0x409/serialnumber", UAC_GADGET_ROOT);
    uac_write_attr(path, "0000001");

    // --- UAC2 function ---
    snprintf(path, sizeof(path), "%s/functions/uac2.0", UAC_GADGET_ROOT);
    if (uac_mkdir(path) < 0 && errno != EEXIST) {
        fprintf(stderr, "uac: cannot create uac2 function: %s\n", strerror(errno));
        return -1;
    }

    // Capture (host reads IQ from us): 2 ch, 24-bit, 48 kHz
    snprintf(path, sizeof(path), "%s/functions/uac2.0/c_srate",  UAC_GADGET_ROOT);
    uac_write_attr(path, "48000");
    snprintf(path, sizeof(path), "%s/functions/uac2.0/c_ssize",  UAC_GADGET_ROOT);
    uac_write_attr(path, "3");           // 3 bytes = 24-bit PCM
    snprintf(path, sizeof(path), "%s/functions/uac2.0/c_chmask", UAC_GADGET_ROOT);
    uac_write_attr(path, "3");           // bitmask: ch0 | ch1  = L+R

    // Playback (host → device, unused but the UAC2 function requires it)
    snprintf(path, sizeof(path), "%s/functions/uac2.0/p_srate",  UAC_GADGET_ROOT);
    uac_write_attr(path, "48000");
    snprintf(path, sizeof(path), "%s/functions/uac2.0/p_ssize",  UAC_GADGET_ROOT);
    uac_write_attr(path, "3");
    snprintf(path, sizeof(path), "%s/functions/uac2.0/p_chmask", UAC_GADGET_ROOT);
    uac_write_attr(path, "3");

    // --- Config c.1 ---
    snprintf(path, sizeof(path), "%s/configs/c.1", UAC_GADGET_ROOT);
    uac_mkdir(path);
    snprintf(path, sizeof(path), "%s/configs/c.1/strings/0x409", UAC_GADGET_ROOT);
    uac_mkdir(path);
    snprintf(path, sizeof(path), "%s/configs/c.1/strings/0x409/configuration",
             UAC_GADGET_ROOT);
    uac_write_attr(path, "Default");
    snprintf(path, sizeof(path), "%s/configs/c.1/bmAttributes", UAC_GADGET_ROOT);
    uac_write_attr(path, "0xC0");        // self-powered + bus-powered
    snprintf(path, sizeof(path), "%s/configs/c.1/MaxPower", UAC_GADGET_ROOT);
    uac_write_attr(path, "250");         // 250 × 2 mA = 500 mA

    // --- Link function into config ---
    char func_abs[256], link_path[256];
    snprintf(func_abs,  sizeof(func_abs),  "%s/functions/uac2.0", UAC_GADGET_ROOT);
    snprintf(link_path, sizeof(link_path), "%s/configs/c.1/uac2.0", UAC_GADGET_ROOT);
    uac_symlink(func_abs, link_path);

    // --- Bind to the UDC ---
    char udc_name[128] = {0};
    if (uac_find_udc(udc_name, sizeof(udc_name)) < 0) {
        fprintf(stderr, "uac: no UDC found — USB gadget not available\n");
        // Not a hard failure: HPSDR P1 continues without UAC
        return -1;
    }
    snprintf(path, sizeof(path), "%s/UDC", UAC_GADGET_ROOT);
    if (uac_write_attr(path, udc_name) < 0) {
        fprintf(stderr, "uac: cannot bind to UDC '%s': %s\n", udc_name, strerror(errno));
        return -1;
    }

    printf("uac: gadget 'sBitx IQ' bound to UDC '%s'\n", udc_name);
    return 0;
}

// Tear down the gadget: unbind from UDC, unlink function, remove configfs nodes.
// A best-effort cleanup — errors are logged but not fatal.
static void uac_gadget_destroy(void) {
    char path[256];

    // Unbind: write empty string to UDC
    snprintf(path, sizeof(path), "%s/UDC", UAC_GADGET_ROOT);
    uac_write_attr(path, "");

    // Remove the function symlink from the config
    snprintf(path, sizeof(path), "%s/configs/c.1/uac2.0", UAC_GADGET_ROOT);
    unlink(path);

    // Remove config strings, config, function strings directories in order
    // (configfs requires directories to be emptied before rmdir)
    char dirs[8][256];
    snprintf(dirs[0], 256, "%s/configs/c.1/strings/0x409",  UAC_GADGET_ROOT);
    snprintf(dirs[1], 256, "%s/configs/c.1",                UAC_GADGET_ROOT);
    snprintf(dirs[2], 256, "%s/functions/uac2.0",           UAC_GADGET_ROOT);
    snprintf(dirs[3], 256, "%s/strings/0x409",              UAC_GADGET_ROOT);
    snprintf(dirs[4], 256, "%s",                            UAC_GADGET_ROOT);
    for (int i = 0; i < 5; i++)
        rmdir(dirs[i]);   // silently tolerate ENOTEMPTY / ENOENT

    printf("uac: gadget removed\n");
}

// ---------------------------------------------------------------------------
// ALSA loopback PCM setup
// ---------------------------------------------------------------------------

// Open and configure the ALSA loopback PCM for write (playback side).
// The UAC2 function driver in the kernel reads the capture side of the same
// loopback card and feeds it to the USB host as the audio stream.
//
// Parameters match the UAC2 descriptor exactly:
//   format   : SND_PCM_FORMAT_S24_3LE  (24-bit packed little-endian)
//   rate     : UAC_RATE    (48000 Hz)
//   channels : UAC_CHANNELS (2)
//
// Returns 0 on success, -1 on ALSA error.
static int uac_alsa_open(void) {
    // Locate the loopback card index
    int card_idx = -1;
    if (uac_find_loopback_card(&card_idx) < 0) {
        fprintf(stderr, "uac: snd-aloop not loaded — run: modprobe snd-aloop\n");
        return -1;
    }

    // Playback side of the loopback: device 0, subdevice 1
    char dev_name[64];
    snprintf(dev_name, sizeof(dev_name), "hw:%d,0,1", card_idx);

    int err;
    if ((err = snd_pcm_open(&uac_pcm_handle, dev_name,
                            SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "uac: snd_pcm_open(%s) failed: %s\n",
                dev_name, snd_strerror(err));
        uac_pcm_handle = NULL;
        return -1;
    }

    snd_pcm_hw_params_t *hw;
    snd_pcm_hw_params_alloca(&hw);
    snd_pcm_hw_params_any(uac_pcm_handle, hw);

    // Interleaved, 24-bit packed LE, 48 kHz, 2 channels
    snd_pcm_hw_params_set_access(uac_pcm_handle, hw,
                                 SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(uac_pcm_handle, hw,
                                 SND_PCM_FORMAT_S24_3LE);
    unsigned int rate = UAC_RATE;
    snd_pcm_hw_params_set_rate_near(uac_pcm_handle, hw, &rate, NULL);
    snd_pcm_hw_params_set_channels(uac_pcm_handle, hw, UAC_CHANNELS);

    snd_pcm_uframes_t period = UAC_PERIOD_FRAMES;
    snd_pcm_hw_params_set_period_size_near(uac_pcm_handle, hw, &period, NULL);
    unsigned int periods = UAC_PERIODS;
    snd_pcm_hw_params_set_periods_near(uac_pcm_handle, hw, &periods, NULL);

    if ((err = snd_pcm_hw_params(uac_pcm_handle, hw)) < 0) {
        fprintf(stderr, "uac: snd_pcm_hw_params failed: %s\n", snd_strerror(err));
        snd_pcm_close(uac_pcm_handle);
        uac_pcm_handle = NULL;
        return -1;
    }

    if ((err = snd_pcm_prepare(uac_pcm_handle)) < 0) {
        fprintf(stderr, "uac: snd_pcm_prepare failed: %s\n", snd_strerror(err));
        snd_pcm_close(uac_pcm_handle);
        uac_pcm_handle = NULL;
        return -1;
    }

    printf("uac: ALSA loopback PCM opened: %s @ %u Hz, 24-bit, %d ch\n",
           dev_name, rate, UAC_CHANNELS);
    return 0;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialise the USB Audio Class gadget and ALSA loopback.
// Call once from the GTK main thread after hpsdr_init().
// Returns 0 if both gadget and PCM are ready, -1 if either step fails.
// Partial failure is handled gracefully: if only the PCM fails, the gadget
// is also torn down to leave the system consistent.
int uac_init(void) {
    if (uac_gadget_create() < 0) {
        // uac_gadget_create already printed the reason
        return -1;
    }
    uac_gadget_up = 1;

    if (uac_alsa_open() < 0) {
        uac_gadget_destroy();
        uac_gadget_up = 0;
        return -1;
    }

    uac_active = 1;
    printf("uac: USB IQ audio stream ready — device name: 'sBitx IQ'\n");
    return 0;
}

// Deliver one 48 kHz IQ sample pair to the USB audio stream.
//
// Called from the sBitx audio thread once per decimated sample (i.e. at
// 48 kHz).  Samples are buffered locally; when UAC_PERIOD_FRAMES frames have
// accumulated, the entire period is written to the ALSA loopback in one call,
// which matches the PCM period size and keeps the write path non-blocking.
//
// i_val and q_val are floating-point normalised to the range [-1.0, +1.0],
// consistent with the rest of hpsdr_p1.c.
//
// The function is a no-op when uac_init() has not succeeded or after uac_stop().
void uac_push_iq(double i_val, double q_val) {
    if (!uac_pcm_handle || !uac_active) return;

    // Clamp to [-1, 1] before conversion
    if (i_val >  1.0) i_val =  1.0;
    if (i_val < -1.0) i_val = -1.0;
    if (q_val >  1.0) q_val =  1.0;
    if (q_val < -1.0) q_val = -1.0;

    // Scale to 24-bit signed integer range and pack as 3-byte little-endian
    // (SND_PCM_FORMAT_S24_3LE: bytes stored as [LSB, mid, MSB])
    int32_t i_int = (int32_t)(i_val * 8388607.0);   // 2^23 - 1
    int32_t q_int = (int32_t)(q_val * 8388607.0);

    uint8_t *slot = uac_pcm_buf + uac_buf_pos * UAC_FRAME_BYTES;
    // I sample (left channel)
    slot[0] = (uint8_t)( i_int        & 0xFF);
    slot[1] = (uint8_t)((i_int >>  8) & 0xFF);
    slot[2] = (uint8_t)((i_int >> 16) & 0xFF);
    // Q sample (right channel)
    slot[3] = (uint8_t)( q_int        & 0xFF);
    slot[4] = (uint8_t)((q_int >>  8) & 0xFF);
    slot[5] = (uint8_t)((q_int >> 16) & 0xFF);

    uac_buf_pos++;

    if (uac_buf_pos >= UAC_BUF_FRAMES) {
        // Flush a full period to the ALSA loopback
        snd_pcm_sframes_t written = snd_pcm_writei(
            uac_pcm_handle, uac_pcm_buf, (snd_pcm_uframes_t)UAC_BUF_FRAMES);

        if (written == -EPIPE) {
            // Buffer underrun: attempt recovery then retry once
            snd_pcm_prepare(uac_pcm_handle);
            snd_pcm_writei(uac_pcm_handle, uac_pcm_buf,
                           (snd_pcm_uframes_t)UAC_BUF_FRAMES);
        } else if (written < 0) {
            // Other ALSA error — log once, then recover
            fprintf(stderr, "uac: snd_pcm_writei error: %s\n",
                    snd_strerror((int)written));
            snd_pcm_recover(uac_pcm_handle, (int)written, 1 /*silent*/);
        }

        uac_buf_pos = 0;
    }
}

// Tear down the UAC2 gadget and release ALSA resources.
// Call from the GTK main thread, paired with uac_init().
// Safe to call even if uac_init() was never called or failed.
void uac_stop(void) {
    uac_active = 0;

    if (uac_pcm_handle) {
        snd_pcm_drain(uac_pcm_handle);
        snd_pcm_close(uac_pcm_handle);
        uac_pcm_handle = NULL;
        printf("uac: ALSA PCM closed\n");
    }

    if (uac_gadget_up) {
        uac_gadget_destroy();
        uac_gadget_up = 0;
    }
}

// Returns 1 while the UAC stream is initialised and ready to accept samples.
// A return value of 0 means uac_init() has not been called, failed, or
// uac_stop() has been called.
int uac_is_active(void) {
    return uac_active;
}
