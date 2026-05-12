// hpsdr_p1.c — HPSDR Protocol 1 interface for sBitx
//
// Provide the interface between the sbitx and an external SDR app using HPSDR Protocol 1.
// The sBitx tries to emulate a HermesLite to work with a number of existing SDR apps.
// Major functions are:
//   Signal processing and buffering:
//    - manage a data buffer to prevent dropping data between the sbitx and external app
//    - Upsample (48k to 96k) using a 6-tap Polyphase FIR filter (Half-band filter)
//      to calculate the midpoint samples
//    - Decimation (96k to 48k) using a 31-tap half-band FIR filter to apply a 24kHz LPF
//      before decimating the signal
//   HPSDR Protocol 1:
//    - identify packet types
//    - add or extract I and Q and other controls and data, and copy in and out of buffer
//   State translation (sBitx <=> SDR application)
//    - coordinate state between sBitx and external SDR app (T/R switch, freq, gain settings,
//      etc.
//   Initialization control and shutdown
//    - sbitx.c needs to start and stop and get status on this interface
//
// There is support for data moving in both directions but the focus has been on receive functions.
// 
// Thanks to Dave N1AI and Juan WP3DN
// Mike KB2ML

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
static void tx_upsample_and_push(double i_val, double q_val);
// Public: hpsdr_tx_iq_active, hpsdr_get_tx_iq, hpsdr_send_iq (defined in .h)

// HPSDR Protocol 1 Implementation (Section 2)
static int hpsdr_classify(const uint8_t *buf, int len);
static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use);
static void build_and_send_packet(void);
static int hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result);
static void reset_all_tx_state(void);
static void handle_command(uint8_t *buf, int len, struct sockaddr_in *sender);

// State Translation (Section 3)
static void apply_freq_from_ep2(uint32_t freq);
static void apply_mox_from_ep2(int mox);
static gboolean hpsdr_tr_idle(gpointer data);

// Initialization, Control & Shutdown (Section 4)
static gboolean hpsdr_watchdog(gpointer data);
static void *hpsdr_poll_thread(void *arg);
// Public: hpsdr_init, hpsdr_stop, hpsdr_is_connected, hpsdr_poll (defined in .h)

// Configuration & Statics
#define HPSDR_PORT 1024
#define HPSDR_PKT_SIZE 1032
#define SAMPLES_PER_PKT 126
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
static double iq_buf_i[SAMPLES_PER_PKT];
static double iq_buf_q[SAMPLES_PER_PKT];
static int iq_buf_count = 0;
static double hpsdr_iq_gain = 1.0;   // add gain to I and Q data going out
static double hpsdr_tx_gain = 1.0;   // add gain to I and Q coming from external SDR app

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

static unsigned long millis_now(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned long)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// Delay lines for I and Q (6 taps)
static double tx_hist_i[6] = {0};
static double tx_hist_q[6] = {0};

static void flush_tx_ring(void) {
  tx_iq_wr = 0;
  tx_iq_rd = 0;

  // Zero out the FIR history buffers
  memset(tx_hist_i, 0, sizeof(tx_hist_i));
  memset(tx_hist_q, 0, sizeof(tx_hist_q));
}

// use 6-tap FIR filter to choose new mid-point
static void tx_upsample_and_push(double i_val, double q_val) {
  // shift delay line
  for (int i = 5; i > 0; i--) {
    tx_hist_i[i] = tx_hist_i[i - 1];
    tx_hist_q[i] = tx_hist_q[i - 1];
  }
  tx_hist_i[0] = i_val;
  tx_hist_q[0] = q_val;

  // Define 6-tap Polyphase Coefficients (Half-band filter)
  // These coefficients are designed for the midpoint sample.
  // They provide a much sharper cutoff at 24kHz than linear averaging.
  static const double taps[6] = {0.0121, -0.0551, 0.2930,
                                 0.2930, -0.0551, 0.0121};
  // calculate the interpolated midpoint (Phase 1)
  double mid_i = 0;
  double mid_q = 0;
  for (int i = 0; i < 6; i++) {
    mid_i += tx_hist_i[i] * taps[i];
    mid_q += tx_hist_q[i] * taps[i];
  }

  // calculate the original sample (Phase 0)
  // In a polyphase upconverter, use the center of the delay line
  // to keep the phase aligned with the calculated midpoint.
  double out_i = tx_hist_i[2];
  double out_q = tx_hist_q[2];

  // Ring buffer logic
  int wr = tx_iq_wr;
  int rd = tx_iq_rd;

  if ((wr - rd) >= (TX_IQ_RING_SIZE - 4))
    return;

  // Sample 1 -The FIR-filtered midpoint
  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = mid_i;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = mid_q;
  wr++;

  // Sample 2 -The aligned original sample
  tx_iq_ring_i[wr & TX_IQ_RING_MASK] = out_i;
  tx_iq_ring_q[wr & TX_IQ_RING_MASK] = out_q;
  wr++;

  tx_iq_wr = wr;
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

// 31-tap half-band FIR coefficients (Fs=96k, Cutoff=24k)
// Note: Every other tap is 0 except for the center tap (index 15)
// Made 'static const' to allow compiler optimizations
static const double hb_coeffs[31] = {
    -0.00055,  0.0,  0.00165,  0.0, -0.00411,  0.0,  0.00877,  0.0, 
    -0.01736,  0.0,  0.03433,  0.0, -0.07612,  0.0,  0.30338,  0.5, 
     0.30338,  0.0, -0.07612,  0.0,  0.03433,  0.0, -0.01736,  0.0, 
     0.00877,  0.0, -0.00411,  0.0,  0.00165,  0.0, -0.00055
};

// History buffers increased to 32 for power-of-two masking (& 31)
static double rx_hist_i[32];
static double rx_hist_q[32];
static int rx_hist_ptr = 0;

// apply 24kHz LPF and decimate 2:1
static void rx_filter_and_decimate(double i0, double i1, double q0, double q1) {
  // Push newest sample pair (i1/q1 then i0/q0)
  rx_hist_ptr = (rx_hist_ptr - 1) & 31;
  rx_hist_i[rx_hist_ptr] = i1;
  rx_hist_q[rx_hist_ptr] = q1;

  rx_hist_ptr = (rx_hist_ptr - 1) & 31;
  rx_hist_i[rx_hist_ptr] = i0;
  rx_hist_q[rx_hist_ptr] = q0;

  // Optimized FIR convolution using symmetry and half-band properties
  // Center tap (index 15) is 0.5
  int p = rx_hist_ptr;
  double filt_i = rx_hist_i[(p + 15) & 31] * 0.5;
  double filt_q = rx_hist_q[(p + 15) & 31] * 0.5;

  // Sum symmetric non-zero taps (0, 2, 4, 6, 8, 10, 12, 14)
  // This reduces multiplications from 31 down to 9
  for (int j = 0; j < 15; j += 2) {
    double c = hb_coeffs[j];
    int idx_low = (p + j) & 31;
    int idx_high = (p + 30 - j) & 31;
    filt_i += (rx_hist_i[idx_low] + rx_hist_i[idx_high]) * c;
    filt_q += (rx_hist_q[idx_low] + rx_hist_q[idx_high]) * c;
  }

  // Store in the HPSDR buffer
  iq_buf_i[iq_buf_count] = filt_i * hpsdr_iq_gain;
  iq_buf_q[iq_buf_count] = filt_q * hpsdr_iq_gain;
  iq_buf_count++;

  if (iq_buf_count >= SAMPLES_PER_PKT) {
    build_and_send_packet();
    iq_buf_count = 0;
  }
}

void hpsdr_send_iq(double *i_samples, double *q_samples, int n) {
    if (!client_active || hpsdr_sock < 0) return;

    // Iterate in steps of 2 to convert 96kHz pairs into 48kHz singles
    for (int k = 0; k < n - 1; k += 2) {
        rx_filter_and_decimate(i_samples[k], i_samples[k+1], 
                               q_samples[k], q_samples[k+1]);
    }
}

// =============================================================================
// HPSDR PROTOCOL 1 IMPLEMENTATION
// =============================================================================
//
// Data flow:
//   Inbound  (SDR app → sBitx): PKT_DISCOVERY, PKT_START, PKT_STOP, PKT_EP2
//   Outbound (sBitx → SDR app): EP6 RX IQ stream
//
// functions below follow this order: classify → inbound handlers → outbound builder
//   → session management → top-level dispatcher

// Protocol statics
static volatile int remote_mox = 0;
static volatile unsigned long ep2_last_time_ms = 0;
#define EP2_WATCHDOG_MS 3500 // only for crash/disconnect

// Packet classification
// Identify the type of every inbound packet before dispatching.
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

// Inbound: Discovery (PKT_DISCOVERY)
// Build a discovery reply. in_use signals whether we already have a client.
static void hpsdr_build_discovery_reply(uint8_t *reply, int in_use) {
  memset(reply, 0, HPSDR_DISCOVERY_REPLY);
  reply[0] = 0xEF;
  reply[1] = 0xFE;
  reply[2] = 0x02;
  reply[3] = in_use ? 0x02 : 0x00;
  
  // MAC Address
  reply[4] = 0x00; 
  reply[5] = 0x1C; reply[6] = 0xC0; reply[7] = 0xA2;
  reply[8] = 0x22; reply[9] = 0x5B;

  reply[10] = 0x06; // Board type: Hermes-Lite
  reply[11] = 0x4A; // Updated firmware version (74 dec) to match reference
  reply[19] = 0x01; // MetisVersion
  reply[20] = 0x01; // NumRxs = 1
}

// Inbound: EP2 (PKT_EP2 — TX IQ + C&C from SDR app)
// Unpack one EP2 packet: extract MOX bit, TX frequency, and TX IQ samples.
// Returns the number of IQ sample pairs unpacked.
static int hpsdr_unpack_ep2(const uint8_t *buf, int len, hpsdr_ep2_result_t *result) {
  if (!buf || len < HPSDR_PKT_SIZE) return 0;

  result->mox = 0;
  result->freq = 0;
  result->tx_freq = 0;
  result->n_samples = 0;

  const uint8_t *ptr = buf + 8; // skip 8-byte Metis header

  for (int frame = 0; frame < 2; frame++) {
    // Check for standard USB sync bytes 7F 7F 7F
    if (ptr[0] != 0x7F || ptr[1] != 0x7F || ptr[2] != 0x7F) {
      ptr += 512;
      continue;
    }

    uint8_t c0 = ptr[3];
    uint8_t addr = (c0 >> 1) & 0x7F; // Command Slot Index
    int mox = c0 & 0x01;             // MOX/PTT bit (Section 8.3)
    
    // The MOX state must be OR'd across both frames (Section 5.5)
    result->mox |= mox;

    // 19-Step Round-Robin Command Decoding (Section 5.4)
    switch (addr) {
      case 0x00: // Case 0: General Settings (Sample Rate, nddc, etc.)
        // Stub: Add logic for Sample Rate (C1 bits 1:0) if needed
        break;

      case 0x01: // Case 1: TX VFO Frequency
        result->tx_freq = ((uint32_t)ptr[4] << 24) | ((uint32_t)ptr[5] << 16) | 
                          ((uint32_t)ptr[6] << 8)  | ((uint32_t)ptr[7]);
        break;

      case 0x02: // Case 2: RX1 (DDC0) Frequency
        result->freq = ((uint32_t)ptr[4] << 24) | ((uint32_t)ptr[5] << 16) | 
                       ((uint32_t)ptr[6] << 8)  | ((uint32_t)ptr[7]);
        break;

      case 0x03: // Case 3: RX2 (DDC1) Frequency
        break;

      case 0x0E: // Case 4: ADC Assignments & TX Step Attenuator (C3)
        break;

      case 0x04: // Case 5: DDC2 Frequency
      case 0x05: // Case 6: DDC3 Frequency
      case 0x06: // Case 7: DDC4 Frequency
      case 0x07: // Case 8: DDC5 Frequency
      case 0x08: // Case 9: DDC6 Frequency
        break;

      case 0x09: // Case 10: Drive Level (C1) & Alex Filters (C3/C4)
        // Stub: This is where sBitx physical filter relays should be updated
        break;

      case 0x0A: // Case 11: Preamp & RX Step Attenuator (C4)
        break;

      case 0x0B: // Case 12: CW Keyer Speed & Weight
        break;

      case 0x0F: // Case 13: CW Enable & Sidetone Level
        break;

      case 0x10: // Case 14: CW Hang Delay & Sidetone Freq
        break;

      case 0x11: // Case 15: EER PWM Settings
        break;

      case 0x12: // Case 16: BPF2 / Transverter
        break;

      // HL2-Specific Extensions (Section 5.3)
      case 0x17: // Case 17: (C0 masked 0x2E) HL2 extension
      case 0x3A: // Case 18: (C0 masked 0x74) HL2 extension
        break;

      default:
        break;
    }

    // Move past the sync + 5 C&C bytes to the IQ payload
    ptr += 8; 

    // Unpack 16-bit Big-Endian IQ Samples (Section 8.4)
    // For nddc=1, there are 63 samples per USB frame.
    for (int j = 0; j < 63 && result->n_samples < SAMPLES_PER_PKT; j++) {
      // Offset 0-3 is L/R Audio (ignored), Offset 4-7 is TX IQ
      int16_t is = (int16_t)(((uint16_t)ptr[4] << 8) | (uint16_t)ptr[5]);
      int16_t qs = (int16_t)(((uint16_t)ptr[6] << 8) | (uint16_t)ptr[7]);

      result->iq[result->n_samples * 2 + 0] = (float)is / 32768.0f * hpsdr_tx_gain;
      result->iq[result->n_samples * 2 + 1] = (float)qs / 32768.0f * hpsdr_tx_gain;

      ptr += 8; // Move to next 8-byte sample group
      result->n_samples++;
    }
  }

  return result->n_samples;
}
// Outbound: EP6 (RX IQ stream to SDR app)
// Build and send one EP6 packet containing two frames of 63 IQ sample pairs,
// plus C&C bytes reporting current sBitx state (T/R status, frequency).
static void build_and_send_packet(void) {
  uint8_t pkt[HPSDR_PKT_SIZE];
  memset(pkt, 0, sizeof(pkt));

  // Metis Header for EP6 (Radio -> Host)
  pkt[0] = 0xEF; pkt[1] = 0xFE; pkt[2] = 0x01; pkt[3] = 0x06;
  pkt[4] = (tx_seq >> 24) & 0xFF;
  pkt[5] = (tx_seq >> 16) & 0xFF;
  pkt[6] = (tx_seq >> 8)  & 0xFF;
  pkt[7] = tx_seq & 0xFF;

  uint32_t seq_for_cc = tx_seq++;

  for (int frame = 0; frame < 2; frame++) {
    uint8_t *fp = pkt + 8 + frame * 512;
    fp[0] = 0x7F; fp[1] = 0x7F; fp[2] = 0x7F; // USB Sync

    // Round-Robin: 5 slots (0-4) as per Capture Reference Section 4.3
    int cc_addr = (seq_for_cc * 2 + frame) % 5;
    
    // C0 byte: Bits 7:3 = Slot Index, Bit 0 = PTT/MOX
    fp[3] = (cc_addr << 3) | (in_tx ? 1 : 0);

    // Map C1-C4 bytes based on the Slot Index
    switch (cc_addr) {
      case 0: // Slot 0: ADC Overload & Version
        fp[4] = 0x00; // C1: Bit 0 is ADC Overload (leave 0 to avoid false Clip)
        fp[5] = 0x00; // C2: Digital Inputs
        fp[6] = 0x4A; // C3: Firmware Version (74 dec)
        fp[7] = 0x00; // C4: Reserved
        break;

      case 1: // Slot 1: Exciter & Forward Power
        // C1-C2: Exciter Power, C3-C4: Forward PA Power
        fp[4] = 0x00; fp[5] = 0x00; 
        fp[6] = 0x00; fp[7] = 0x00; 
        break;

      case 2: // Slot 2: Reverse Power & PA Voltage
        // C1-C2: Reverse PA Power, C3-C4: PA Volts (User_ADC0)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;

      case 3: // Slot 3: PA Current & Supply Voltage
        // C1-C2: PA Amps (User_ADC1), C3-C4: Supply Volts
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;

      case 4: // Slot 4: Additional ADC Overload Flags
        // C1: ADC0, C2: ADC1, C3: ADC2 (per Section 4.3)
        fp[4] = 0x00; fp[5] = 0x00;
        fp[6] = 0x00; fp[7] = 0x00;
        break;
    }

    // send I and Q data frames, along with MIC data
    // 63 IQ sample pairs per frame, packed as 24-bit big-endian
    for (int s = 0; s < 63; s++) {
      int idx = frame * 63 + s;
      uint8_t *sp = fp + 8 + s * 8;

      int32_t i_val = (int32_t)(iq_buf_i[idx] * 559240.0);
      if (i_val > 8388607)  i_val = 8388607;
      if (i_val < -8388608) i_val = -8388608;

      int32_t q_val = (int32_t)(iq_buf_q[idx] * 559240.0);
      if (q_val > 8388607)  q_val = 8388607;
      if (q_val < -8388608) q_val = -8388608;

      sp[0] = (i_val >> 16) & 0xFF; // I
      sp[1] = (i_val >> 8)  & 0xFF;
      sp[2] =  i_val        & 0xFF;
      sp[3] = (q_val >> 16) & 0xFF; // Q
      sp[4] = (q_val >> 8)  & 0xFF;
      sp[5] =  q_val        & 0xFF;
      sp[6] = 0; // Mic (unused)
      sp[7] = 0;
    }
  }

  if (client_active) {
    sendto(hpsdr_sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&stream_dest, sizeof(stream_dest));
  }
}

// Session management
// Reset all TX-related state. Called on PKT_START, PKT_STOP, and watchdog timeout.
static void reset_all_tx_state(void) {
  flush_tx_ring();
  hpsdr_tx_data_active = 0;
  remote_mox = 0;
  ep2_last_time_ms = millis_now();
}

// Top-level packet dispatcher
// Classify each inbound packet and route it to the appropriate handler.
// For EP2, state translation functions are called after unpacking.
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
    if (!client_active) break;
    ep2_last_time_ms = millis_now();
    
    hpsdr_ep2_result_t r;
    hpsdr_unpack_ep2(buf, len, &r);

    apply_freq_from_ep2((r.mox && r.tx_freq) ? r.tx_freq : r.freq);
    apply_mox_from_ep2(r.mox);

    if (r.n_samples > 0) {
      for (int k = 0; k < r.n_samples; k++) {
        tx_upsample_and_push(r.iq[k * 2], r.iq[k * 2 + 1]);
      }
    }
    break;
  }

  } // switch end
}

// =============================================================================
// STATE TRANSLATION
// =============================================================================
// These functions translate state from HPSDR Protocol 1 into sBitx state.
// Future additions: gain settings, band info, antenna selection, etc.

// Apply a frequency received from an EP2 packet to the sBitx.
static void apply_freq_from_ep2(uint32_t freq) {
  if (freq > 0 && freq != (uint32_t)freq_hdr) {
    printf("hpsdr: remote set freq %d Hz\n", freq);
    char cmd[50];
    sprintf(cmd, "freq %d", freq);
    remote_execute(cmd);
  }
}

// Translate the MOX bit from EP2 into a debounced sBitx T/R switch action.
// Requires 4 consecutive consistent packets (~10 ms) before committing.
static void apply_mox_from_ep2(int mox) {
  static int mox_count = 0;
  static int mox_pending_state = 0;

  if (mox != remote_mox) {
    // MOX differs from current state — count consecutive matches
    if (mox != mox_pending_state) {
      // Direction changed again — restart counter
      mox_pending_state = mox;
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

// GTK idle callback to apply a pending T/R switch on the GTK main thread.
// tr_pending: 0 = nothing, 1 = tx_on pending, 2 = tx_off pending
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

// =============================================================================
// INITIALIZATION, CONTROL & SHUTDOWN
// =============================================================================

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
