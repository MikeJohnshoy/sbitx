// tci_sbitx.c
//
// sbitx's side of the TCI server: tci_radio.h for tci.c, the TCI and
// TCI_PORT settings, the receive streams and transmit audio.
// docs/tci_design_study.md.
//
// THREADS
//
// sbitx's controls belong to the GTK thread, and TCI's commands arrive on
// network threads, so the two never touch each other directly:
//
//   - The GTK thread publishes the state TCI reports (frequency, mode, TX,
//     drive, volume, mute, RIT, CW speed, the S-meter) every 10 ms, under
//     a lock, from tci_sbitx_tick(); tci_radio_get() copies it out.
//   - A set is queued with remote_execute(), which the GTK thread runs on
//     its next tick, as the web UI, HPSDR and the 8081 console do. It uses
//     sbitx's own commands: freq, mode, and setfield, which sets a field as
//     the screen would.
//
// A set is reported at once: until the published state shows it (or for
// PENDING_MS, if sbitx took another value), tci_radio_get() reports the
// value asked for, so tci.c's echo is the new value and the next sample
// doesn't send the old one back first.
//
// MODES
//
// TCI offers USB, LSB, CW, CWR and DIGU. DIGU is sbitx's DIGI, the
// wideband setting for an external program's FT8, AM, FM and the rest, and
// sbitx's own wideband modes (FT8, FT4, AM, FM) read as DIGU too: sbitx
// goes on receiving, demodulating and showing them while the program works
// in parallel. So DIGU asked for in one of them changes nothing, and in any
// other mode selects DIGI. 2TONE reads as USB.
//
// RECEIVE STREAMS
//
// The demodulated audio is decimated from 96 to 48 kHz and scaled so int32
// full scale is 1, the level the loopback gives WSJT-X. The I/Q is the
// mixer's filtered output, the block HPSDR gets (sound_process(), sbitx.c),
// with I and Q as they are: HPSDR swaps them, which mirrors the spectrum
// the way SparkSDR expects, so as they are, a station above the dial is at
// a positive frequency. It is then shifted (sbitx.c works out by how much)
// to be centred where the clients put dds rather than on sbitx's receive
// frequency, and in CW and CWR by the pitch the mixer puts a zero-beat
// signal at, so a client's frequency scale is the true one. While
// transmitting both are silence.
//
// CW TEXT
//
// cw_macros text is translated to the keyer's text (tci_cw.c) on the TCI
// thread and waits here; the GTK thread moves it into the TEXT field, the
// keyer's queue, as a macro would, as much as the field has room for each
// tick, so text of any length goes in order. It is sent in CW and CWR only:
// asked for in another mode it is refused, and what is waiting is dropped
// if the mode leaves CW. cw_macros_stop, or the key or paddle closing while
// TCI text is being sent - the operator taking over - empties the queue and
// stops the character being sent (cw_text_clear()); either way the speed
// goes back to WPM.
//
// TRANSMIT
//
// A client's trx keys sbitx with its own commands: t to transmit, r to
// receive, and rtx in DIGI, FT8 and FT4 when the client doesn't send its
// own audio, which takes the audio from the loopback as rigctld's PTT does
// for WSJT-X. With its own audio (trx:0,true,tci), that audio replaces
// whatever sbitx would send in USB, LSB, DIGI, FT8 and FT4: it is pulled
// at 48 kHz, upsampled to 96 kHz and scaled so its full scale is the level
// of sbitx's own FT8 tones (TX_AUDIO_SCALE), and goes into tx_process() as
// the microphone would,
// without the voice processing (compression, EQ, CESSB, the voice clip),
// since it is a program's line audio. The level matters because JTDX in
// TCI mode sends its audio at full scale and turns its power slider into
// drive: commands, which set DRIVE, so power is set as for sbitx's own FT8. In AM, FM, CW and CWR, which take
// their audio at other levels or none, a trx with its own audio is
// refused; in 2TONE, any trx. While a client holds TX, and until sbitx has
// run the r that ends it, sbitx's own FT8/FT4 modem steps aside (modems.c):
// it neither ends the transmission nor sends its own; and for the last
// blocks before r runs, a client's own-audio TX sends silence rather than
// the microphone.

#include "tci_sbitx.h"
#include "decim48k.h"
#include "sdr.h"
#include "sdr_ui.h"
#include "tci.h"
#include "tci_radio.h"
#include "modem_cw.h"
#include "tci_cw.h"
#include "tci_stream.h"
#include "upsample48k.h"
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PUBLISH_TICKS 10   // ui_tick runs every 1 ms: state every 10 ms
#define SETTINGS_TICKS 100 // the TCI setting every 100 ms
#define RETRY_MS 10000     // after a failed start, wait this long to try again
#define PENDING_MS 1000
#define BLOCK_MAX 2048
#define IQ_SHIFT_MAX 48000 // half the I/Q's width at 96 kHz
// sbitx's own FT8 sends its tones into tx_process() at 1/21 of full scale
// (ft8_next_sample()'s /7, tx_process()'s /3), and DRIVE is calibrated for
// that level. A program's full scale is made the same; tx_process() divides
// input_mic by 2e9 in the modes that take it.
#define TX_AUDIO_SCALE (2.0e9 / 21.0)
// A transmission's first and last blocks go without the client's audio
// while keying catches up; report a shortfall only beyond that.
#define TX_SHORT_REPORT_MS 100

extern int in_tx;
extern int audio_muted;         // sbitx_gtk.c: MUTE sets AUDIO to 0 and
extern char premute_volume[16]; // keeps the level to go back to here
extern int calculate_s_meter(struct rx *r, double rx_gain);

static pthread_mutex_t state_lock = PTHREAD_MUTEX_INITIALIZER;
static struct tci_radio_state published;
static char published_mode[16] = "USB";

struct pending {
  int active, value;
  long long until;
};
static struct pending p_freq, p_mode, p_drive, p_volume, p_mute, p_wpm, p_tx;
static atomic_int tx_short; // TX audio samples the client didn't send in time
#define CW_WAITING_MAX 1024
#define CW_MACRO_START '\x1f' // in cw_waiting, before each macro's text
static pthread_mutex_t cw_lock = PTHREAD_MUTEX_INITIALIZER;
static char cw_waiting[CW_WAITING_MAX]; // keyer text not yet in the TEXT field
static int cw_waiting_len;
static atomic_int cw_stop_asked;

// A client's TX has been unkeyed, and sbitx hasn't yet run the r that ends
// it: the audio thread sends silence rather than the microphone, and
// sbitx's FT8 still stands aside. Cleared by the GTK thread once in_tx is 0.
static atomic_int tx_finishing, tx_was_own_audio;

static long long now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* ---- Modes -------------------------------------------------------------- */

static int is_wideband(const char *sbitx_mode) {
  return !strcmp(sbitx_mode, "DIGI") || !strcmp(sbitx_mode, "FT8") ||
         !strcmp(sbitx_mode, "FT4") || !strcmp(sbitx_mode, "AM") || !strcmp(sbitx_mode, "FM");
}

static enum tci_mode tci_mode_of(const char *sbitx_mode) {
  if (!strcmp(sbitx_mode, "LSB"))
    return TCI_MODE_LSB;
  if (!strcmp(sbitx_mode, "CW"))
    return TCI_MODE_CW;
  if (!strcmp(sbitx_mode, "CWR"))
    return TCI_MODE_CWR;
  if (is_wideband(sbitx_mode))
    return TCI_MODE_DIGU;
  return TCI_MODE_USB;
}

/* ---- tci_radio.h, on TCI's threads ---------------------------------------- */

static void take(struct pending *p, int *field) {
  if (!p->active)
    return;
  if (*field == p->value || now_ms() > p->until)
    p->active = 0;
  else
    *field = p->value;
}

void tci_radio_get(struct tci_radio_state *s) {
  pthread_mutex_lock(&state_lock);
  *s = published;
  int mode = (int)s->mode;
  take(&p_freq, &s->freq);
  take(&p_mode, &mode);
  take(&p_drive, &s->drive);
  take(&p_volume, &s->volume);
  take(&p_mute, &s->mute);
  take(&p_wpm, &s->wpm);
  take(&p_tx, &s->tx);
  s->mode = (enum tci_mode)mode;
  pthread_mutex_unlock(&state_lock);
}

static void hold(struct pending *p, int value) {
  pthread_mutex_lock(&state_lock);
  p->active = 1;
  p->value = value;
  p->until = now_ms() + PENDING_MS;
  pthread_mutex_unlock(&state_lock);
}

static void setfield(const char *id, const char *value) {
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "setfield %s %s", id, value);
  remote_execute(cmd);
}

static void setfield_int(const char *id, int value) {
  char v[16];
  snprintf(v, sizeof(v), "%d", value);
  setfield(id, v);
}

void tci_radio_set_freq(int hz) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "freq %d", hz);
  hold(&p_freq, hz);
  remote_execute(cmd);
}

void tci_radio_set_mode(enum tci_mode m) {
  static const char *const sbitx_mode[] = {"USB", "LSB", "CW", "CWR", "DIGI"};
  char now[16], cmd[32];
  pthread_mutex_lock(&state_lock);
  strcpy(now, published_mode);
  pthread_mutex_unlock(&state_lock);
  if (m == TCI_MODE_DIGU && is_wideband(now))
    return; // already in the wideband group: sbitx keeps its own mode
  if (tci_mode_of(now) == m && !strcmp(now, sbitx_mode[m]))
    return;
  snprintf(cmd, sizeof(cmd), "mode %s", sbitx_mode[m]);
  hold(&p_mode, (int)m);
  remote_execute(cmd);
}

void tci_radio_set_drive(int drive) {
  hold(&p_drive, drive);
  setfield_int("tx_power", drive);
}

void tci_radio_set_volume(int percent) {
  hold(&p_volume, percent);
  setfield_int("r1:volume", percent);
}

void tci_radio_set_mute(int on) {
  hold(&p_mute, on != 0);
  setfield("#mute", on ? "ON" : "OFF");
}

void tci_radio_set_wpm(int wpm) {
  hold(&p_wpm, wpm);
  setfield_int("#tx_wpm", wpm);
}

static int is_ftx_or_digi(const char *sbitx_mode) {
  return !strcmp(sbitx_mode, "DIGI") || !strcmp(sbitx_mode, "FT8") || !strcmp(sbitx_mode, "FT4");
}

// The modes a client's own audio is sent in (and tci_sbitx_tx_block()'s).
static int takes_audio(const char *sbitx_mode) {
  return is_ftx_or_digi(sbitx_mode) || !strcmp(sbitx_mode, "USB") || !strcmp(sbitx_mode, "LSB");
}

int tci_radio_set_tx(int on, int own_audio) {
  char now[16];
  pthread_mutex_lock(&state_lock);
  strcpy(now, published_mode);
  pthread_mutex_unlock(&state_lock);
  if (on && (!strcmp(now, "2TONE") || (own_audio && !takes_audio(now)))) {
    printf("tci: TX refused - %s doesn't send %s\n", now,
           own_audio ? "a program's audio" : "for a program");
    return -1;
  }
  hold(&p_tx, on != 0);
  if (on) {
    atomic_store(&tx_finishing, 0);
    atomic_store(&tx_was_own_audio, own_audio != 0);
    remote_execute(!own_audio && is_ftx_or_digi(now) ? "rtx" : "t"); // rtx: the loopback's audio
  } else {
    atomic_store(&tx_finishing, 1);
    remote_execute("r");
  }
  return 0;
}

int tci_sbitx_tx_engaged(void) { return tci_holds_tx() || atomic_load(&tx_finishing); }

static int is_cw(const char *sbitx_mode) { return !strcmp(sbitx_mode, "CW") || !strcmp(sbitx_mode, "CWR"); }

void tci_radio_cw_text(const char *text) {
  char now[16], keyer[CW_WAITING_MAX], skipped[32];
  pthread_mutex_lock(&state_lock);
  strcpy(now, published_mode);
  pthread_mutex_unlock(&state_lock);
  if (!is_cw(now)) {
    printf("tci: CW text ignored - sbitx is in %s, and sends it in CW and CWR only\n", now);
    return;
  }
  int n = tci_cw_translate(text, keyer, sizeof(keyer), skipped, sizeof(skipped));
  if (skipped[0])
    printf("tci: CW text: left out, not in sbitx's Morse table: %s\n", skipped);
  if (strspn(keyer, (char[]){CW_SPEED_DOWN, CW_SPEED_UP, '\0'}) == (size_t)n)
    return; // nothing to send
  pthread_mutex_lock(&cw_lock);
  if (cw_waiting_len + 1 + n < CW_WAITING_MAX) {
    cw_waiting[cw_waiting_len++] = CW_MACRO_START;
    memcpy(cw_waiting + cw_waiting_len, keyer, (size_t)n + 1);
    cw_waiting_len += n;
    printf("tci: CW text: %s\n", text);
  } else {
    printf("tci: CW text refused - too much waiting to be sent\n");
  }
  pthread_mutex_unlock(&cw_lock);
}

void tci_radio_cw_stop(void) { atomic_store(&cw_stop_asked, 1); }

/* ---- The GTK thread ---------------------------------------------------------- */

static double strength_dbm(void) {
  if (!rx_list)
    return -127.0;
  int v = calculate_s_meter(rx_list, (double)get_rx_gain());
  int s = v / 100, add = v % 100;
  // S9 is -73 dBm and an S-unit 6 dB; above S9, add is the dB over it
  return s >= 9 ? -73.0 + add : -127.0 + 6.0 * s + add;
}

static int on_off(const char *label) {
  const char *v = field_str(label);
  return v && !strcmp(v, "ON");
}

static void publish(void) {
  struct tci_radio_state s;
  const char *mode = field_str("MODE");
  s.freq = field_int("FREQ");
  s.mode = tci_mode_of(mode ? mode : "USB");
  s.tx = in_tx != 0;
  s.drive = field_int("DRIVE");
  // while muted, the level MUTE OFF goes back to, so a program's volume
  // control doesn't drop to the bottom
  s.volume = audio_muted && premute_volume[0] ? atoi(premute_volume) : field_int("AUDIO");
  s.mute = on_off("MUTE");
  s.rit_on = on_off("RIT");
  s.rit = field_int("RIT_DELTA");
  s.wpm = field_int("WPM");
  s.dbm = strength_dbm();
  pthread_mutex_lock(&state_lock);
  published = s;
  snprintf(published_mode, sizeof(published_mode), "%s", mode ? mode : "USB");
  pthread_mutex_unlock(&state_lock);
}

static void follow_settings(void) {
  static int port_in_use;
  static long long retry_at;
  int want = on_off("TCI");
  int port = field_int("TCI_PORT");
  if (tci_running() && (!want || port != port_in_use))
    tci_stop();
  if (want && !tci_running() && now_ms() >= retry_at) {
    publish(); // so the first client gets the state as it is
    if (tci_init("", port, TCI_DEFAULT_MAX_CLIENTS) == 0) {
      port_in_use = port;
    } else {
      printf("tci: can't listen on port %d - trying again in %d s\n", port, RETRY_MS / 1000);
      retry_at = now_ms() + RETRY_MS;
    }
  }
}

// Once a transmission is over, how much of its audio the client was late
// with, if more than keying accounts for.
static void report_tx_short(void) {
  if (tci_holds_tx())
    return;
  int n = atomic_exchange(&tx_short, 0);
  if (n > TX_SHORT_REPORT_MS * 48)
    printf("tci: the client's TX audio was %d ms short\n", n / 48);
}

static void cw_drop_waiting(void) {
  pthread_mutex_lock(&cw_lock);
  cw_waiting_len = 0;
  pthread_mutex_unlock(&cw_lock);
}

// Moves what fits of the waiting text into the TEXT field. A macro that
// follows one still in the field gets a word space before it, so the two
// don't run together. Returns how much was taken from cw_waiting.
static int cw_feed(void) {
  static char last = ' '; // the last character given to the field
  int taken = 0;
  while (taken < cw_waiting_len) {
    char *p = cw_waiting + taken;
    if (*p == CW_MACRO_START) {
      if (get_tx_data_length() > 0 && last != ' ' && p[1] != ' ') {
        if (cw_text_append(" ") == 0)
          break; // no room yet
        last = ' ';
      }
      taken++;
      continue;
    }
    int len = (int)strcspn(p, (char[]){CW_MACRO_START, '\0'});
    char chunk[CW_WAITING_MAX];
    memcpy(chunk, p, (size_t)len);
    chunk[len] = '\0';
    int n = cw_text_append(chunk);
    if (n > 0)
      last = chunk[n - 1];
    taken += n;
    if (n < len)
      break; // the field is full
  }
  return taken;
}

// Moves waiting CW text into the TEXT field, and stops it when asked or
// when the operator keys.
static void follow_cw_text(void) {
  static int sending;       // TCI text is waiting or in the TEXT field
  static unsigned presses;  // the key's closures when it started
  const char *mode = field_str("MODE");
  int in_cw = mode && is_cw(mode);
  int stop = atomic_exchange(&cw_stop_asked, 0);
  if (stop || (sending && cw_key_press_count() != presses) || (sending && !in_cw)) {
    cw_drop_waiting();
    if (sending && in_cw) // outside CW, sbitx's mode change has cleared the field
      cw_text_clear();
    if (sending)
      printf("tci: CW text %s\n", stop ? "stopped" : in_cw ? "stopped by the key" : "dropped - not in CW");
    sending = 0;
    return;
  }
  pthread_mutex_lock(&cw_lock);
  if (cw_waiting_len && in_cw) {
    if (!sending) {
      sending = 1;
      presses = cw_key_press_count();
    }
    int n = cw_feed();
    memmove(cw_waiting, cw_waiting + n, (size_t)(cw_waiting_len - n) + 1);
    cw_waiting_len -= n;
  }
  int waiting = cw_waiting_len;
  pthread_mutex_unlock(&cw_lock);
  if (sending && !waiting && get_tx_data_length() == 0)
    sending = 0; // all sent
}

void tci_sbitx_tick(void) {
  static int ticks;
  ticks++;
  if (!in_tx && atomic_load(&tx_finishing))
    atomic_store(&tx_finishing, 0); // the r has run
  if (tci_running())
    follow_cw_text();
  if (ticks % SETTINGS_TICKS == 0)
    follow_settings();
  if (ticks % PUBLISH_TICKS == 0 && tci_running()) {
    publish();
    report_tx_short();
  }
}

/* ---- The audio thread ---------------------------------------------------------- */

void tci_sbitx_rx_block(const int32_t *speaker, const double *i, const double *q, int n,
                        int shift_hz) {
  static struct decim48k_state dec;
  static const double zeros[BLOCK_MAX];
  static double rot_re = 1.0, rot_im = 0.0; // the I/Q shift's phase, carried on
  double audio[BLOCK_MAX / 2 + 1], si[BLOCK_MAX], sq[BLOCK_MAX];
  int m = 0;
  if (!tci_running())
    return;
  if (n > BLOCK_MAX)
    n = BLOCK_MAX;
  for (int k = 0; k < n; k++) {
    double x = speaker ? speaker[k] / 2147483648.0 : 0.0;
    double y;
    if (decim48k_apply(&dec, x, &y))
      audio[m++] = y;
  }
  tci_push_audio_rx(audio, m);
  if (!i || !q || shift_hz == 0 || shift_hz < -IQ_SHIFT_MAX || shift_hz > IQ_SHIFT_MAX) {
    // beyond the I/Q's width (a retune catching up), silence
    int off_range = i && q && shift_hz != 0;
    tci_push_iq(i && !off_range ? i : zeros, q && !off_range ? q : zeros, n);
    return;
  }
  // multiply by e^(j 2 pi shift t), by a rotation per sample
  double step_re = cos(2 * M_PI * shift_hz / 96000.0), step_im = sin(2 * M_PI * shift_hz / 96000.0);
  for (int k = 0; k < n; k++) {
    si[k] = i[k] * rot_re - q[k] * rot_im;
    sq[k] = i[k] * rot_im + q[k] * rot_re;
    double re = rot_re * step_re - rot_im * step_im;
    rot_im = rot_re * step_im + rot_im * step_re;
    rot_re = re;
  }
  double mag = sqrt(rot_re * rot_re + rot_im * rot_im); // keep it on the unit circle
  rot_re /= mag;
  rot_im /= mag;
  tci_push_iq(si, sq, n);
}

int tci_sbitx_tx_block(int32_t *mic, int n, int mode) {
  static struct upsample48k_state up;
  double in48[BLOCK_MAX / 2 + 1];
  int takes = mode == MODE_DIGITAL || mode == MODE_FT8 || mode == MODE_FT4 || mode == MODE_USB ||
              mode == MODE_LSB;
  if (takes && !tci_tx_audio_owned() && atomic_load(&tx_finishing) &&
      atomic_load(&tx_was_own_audio)) {
    memset(mic, 0, (size_t)n * sizeof(*mic)); // the last blocks before r runs
    tci_tx_audio_idle();
    return 1;
  }
  if (!takes || !tci_tx_audio_owned()) {
    tci_tx_audio_idle();
    return 0;
  }
  if (n > BLOCK_MAX)
    n = BLOCK_MAX;
  int need = (n + 1) / 2;
  int got = tci_pull_audio_tx(in48, need);
  if (got < need)
    atomic_fetch_add(&tx_short, need - got);
  int o = 0;
  for (int k = 0; k < need; k++) {
    double out2[2];
    // past what the client sent, silence, so the filter's history runs on
    upsample48k_apply(&up, k < got ? in48[k] : 0.0, out2);
    for (int j = 0; j < 2 && o < n; j++) {
      double x = out2[j] > 1.0 ? 1.0 : out2[j] < -1.0 ? -1.0 : out2[j];
      mic[o++] = (int32_t)lrint(x * TX_AUDIO_SCALE);
    }
  }
  return 1;
}

void tci_sbitx_tx_idle(void) {
  // keyed but not yet transmitting: keep what the client has sent so far
  if (!tci_holds_tx())
    tci_tx_audio_idle();
}
