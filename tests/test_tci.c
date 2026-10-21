// test_tci.c
//
// Bench harness for the TCI server (tci.c, tci_stream.c, tci_ws.c) over
// loopback, with the radio stubbed (tci_radio.h): the initialization burst
// and its form, the client count, echoes of every set to every client, reads answered to the
// asker alone, setting frequency, mode, drive, volume, mute and CW speed,
// dds and if as a panadapter tunes inside its panorama,
// RIT refused with the state in force, a change made elsewhere reaching
// every client, several commands to a message, the client limit, the
// receive audio and I/Q streams (I and Q as given), CW text (cw_macros and
// cw_macros_stop reaching the radio, the text as sent), transmit - who holds
// TX, the client's own audio through TX_CHRONO, a trx the radio refuses,
// release on trx false, disconnect, TX ending elsewhere and stopping - stopping and starting
// again, and decim48k's and upsample48k's response. The harness plays the
// audio thread itself.
// Built and run by make test, as /tmp/sbitx-test_tci.

#define _GNU_SOURCE
#include "decim48k.h"
#include "upsample48k.h"
#include "tci.h"
#include "tci_radio.h"
#include "tci_stream.h"
#include "ws_test_client.h"
#include <ctype.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int failures;
#define CHECK(cond, ...)                                                                           \
  do {                                                                                             \
    if (cond) {                                                                                    \
      printf("  ok   ");                                                                           \
    } else {                                                                                       \
      printf("  FAIL ");                                                                           \
      failures++;                                                                                  \
    }                                                                                              \
    printf(__VA_ARGS__);                                                                           \
    printf("\n");                                                                                  \
  } while (0)

/* ---- The radio, stubbed ----------------------------------------------------- */

// A set takes effect at once, as sbitx's would by the next tick; tci.c
// only sees the state through tci_radio_get().
static pthread_mutex_t radio_lock = PTHREAD_MUTEX_INITIALIZER;
static struct tci_radio_state radio = {14074000, TCI_MODE_USB, 0, 40, 60, 0, 0, 0, 12, -97.0};
static int set_calls;

void tci_radio_get(struct tci_radio_state *s) {
  pthread_mutex_lock(&radio_lock);
  *s = radio;
  pthread_mutex_unlock(&radio_lock);
}

#define STUB_SET(field, v)                                                                         \
  do {                                                                                             \
    pthread_mutex_lock(&radio_lock);                                                               \
    radio.field = (v);                                                                             \
    set_calls++;                                                                                   \
    pthread_mutex_unlock(&radio_lock);                                                             \
  } while (0)

void tci_radio_set_freq(int hz) { STUB_SET(freq, hz); }
void tci_radio_set_mode(enum tci_mode m) { STUB_SET(mode, m); }
void tci_radio_set_drive(int drive) { STUB_SET(drive, drive); }
void tci_radio_set_volume(int percent) { STUB_SET(volume, percent); }
void tci_radio_set_mute(int on) { STUB_SET(mute, on != 0); }
void tci_radio_set_wpm(int wpm) { STUB_SET(wpm, wpm); }

static int tx_calls, tx_own_audio, tx_refuse;
int tci_radio_set_tx(int on, int own_audio) {
  pthread_mutex_lock(&radio_lock);
  int refused = on && tx_refuse;
  if (!refused) {
    radio.tx = on != 0;
    tx_calls++;
    tx_own_audio = own_audio;
  }
  pthread_mutex_unlock(&radio_lock);
  return refused ? -1 : 0;
}

static char cw_text[600];
static int cw_texts, cw_stops;
void tci_radio_cw_text(const char *text) {
  pthread_mutex_lock(&radio_lock);
  snprintf(cw_text, sizeof(cw_text), "%s", text);
  cw_texts++;
  pthread_mutex_unlock(&radio_lock);
}
void tci_radio_cw_stop(void) {
  pthread_mutex_lock(&radio_lock);
  cw_stops++;
  pthread_mutex_unlock(&radio_lock);
}

static int tx_calls_now(void) {
  pthread_mutex_lock(&radio_lock);
  int n = tx_calls;
  pthread_mutex_unlock(&radio_lock);
  return n;
}

static struct tci_radio_state radio_now(void) {
  struct tci_radio_state s;
  tci_radio_get(&s);
  return s;
}

/* ---- Client side ------------------------------------------------------------ */

static long long now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static int readable(int fd, int ms) {
  struct pollfd p = {fd, POLLIN, 0};
  return poll(&p, 1, ms) > 0;
}

// The next frame within ms: its opcode, or -1. Text in *text (malloc'd).
static int next_frame(int fd, int ms, uint8_t **data, size_t *len) {
  if (!readable(fd, ms))
    return -1;
  return recv_frame(fd, data, len, NULL);
}

static void send_text(int fd, const char *s) { send_frame(fd, 1, 1, s, strlen(s), 1); }

// Reads text messages for up to ms, skipping binary frames, until one equals
// want. Returns 1 if it came.
static int expect(int fd, const char *want, int ms) {
  long long end = now_ms() + ms;
  for (long long left = ms; left > 0; left = end - now_ms()) {
    uint8_t *d = NULL;
    size_t n;
    int op = next_frame(fd, (int)left, &d, &n);
    if (op < 0) {
      free(d);
      return 0;
    }
    int hit = op == 1 && !strcmp((char *)d, want);
    free(d);
    if (hit)
      return 1;
  }
  return 0;
}

// Text messages that arrive within ms, newline-separated, into buf.
static int collect(int fd, int ms, char *buf, size_t cap) {
  int count = 0;
  size_t used = 0;
  buf[0] = 0;
  long long end = now_ms() + ms;
  for (long long left = ms; left > 0; left = end - now_ms()) {
    uint8_t *d = NULL;
    size_t n;
    int op = next_frame(fd, (int)left, &d, &n);
    if (op < 0) {
      free(d);
      break;
    }
    if (op == 1) {
      used += (size_t)snprintf(buf + used, used < cap ? cap - used : 0, "%s\n", (char *)d);
      count++;
    }
    free(d);
  }
  return count;
}

static void drain(int fd, int ms) {
  char junk[16384];
  collect(fd, ms, junk, sizeof(junk));
}

// Connects and reads the burst up to ready; into burst (newline-separated).
static int connect_client(char *burst, size_t cap, int *count, long long *took_ms) {
  long long t0 = now_ms();
  int fd = ws_connect();
  if (fd < 0)
    return -1;
  size_t used = 0;
  int n = 0;
  burst[0] = 0;
  for (;;) {
    uint8_t *d = NULL;
    size_t len;
    int op = next_frame(fd, 1500, &d, &len);
    if (op < 0) {
      free(d);
      break;
    }
    if (op == 1) {
      used += (size_t)snprintf(burst + used, used < cap ? cap - used : 0, "%s\n", (char *)d);
      n++;
      int ready = !strcmp((char *)d, "ready;");
      free(d);
      if (ready)
        break;
    } else {
      free(d);
    }
  }
  if (count)
    *count = n;
  if (took_ms)
    *took_ms = now_ms() - t0;
  return fd;
}

static int quick_connect(void) {
  char burst[8192];
  return connect_client(burst, sizeof(burst), NULL, NULL);
}

static uint32_t get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static float get_f32(const uint8_t *p) {
  uint32_t v = get_u32(p);
  float f;
  memcpy(&f, &v, 4);
  return f;
}

// The next binary frame of the given stream type within ms; its payload.
static uint8_t *next_stream(int fd, uint32_t type, int ms, size_t *len) {
  long long end = now_ms() + ms;
  for (long long left = ms; left > 0; left = end - now_ms()) {
    uint8_t *d = NULL;
    int op = next_frame(fd, (int)left, &d, len);
    if (op < 0) {
      free(d);
      return NULL;
    }
    if (op == 2 && *len >= TCI_HEADER_BYTES && get_u32(d + 24) == type)
      return d;
    free(d);
  }
  return NULL;
}

/* ---- Tests -------------------------------------------------------------------- */

static void test_burst(void) {
  printf("initialization burst\n");
  char burst[8192];
  int count = 0;
  long long took = 0;
  int fd = connect_client(burst, sizeof(burst), &count, &took);
  CHECK(fd >= 0, "connects and handshakes");
  if (fd < 0)
    return;
  CHECK(!strncmp(burst, "protocol:ExpertSDR3,2.0;\ndevice:sbitx;\n", 39),
        "protocol first, then device:sbitx");
  char *start = strstr(burst, "\nstart;\n"), *ready = strstr(burst, "\nready;\n");
  CHECK(start && ready && start < ready, "start; before ready;");
  CHECK(ready && ready[8] == 0, "ready; ends it");
  CHECK(count > 20 && count < 256, "%d messages, under Hamlib's 256", count);
  CHECK(took < 1500, "all of it in %lld ms", took);
  int one_each = 1, lower = 1;
  for (char *line = burst; *line;) {
    char *nl = strchr(line, '\n');
    *nl = 0;
    char *semi = strchr(line, ';');
    if (!semi || semi[1] != 0)
      one_each = 0;
    for (char *p = line; *p && *p != ':' && *p != ';'; p++)
      if (isupper((unsigned char)*p))
        lower = 0;
    *nl = '\n';
    line = nl + 1;
  }
  CHECK(one_each, "one command per message, each ending in ;");
  CHECK(lower, "keywords in lowercase");
  CHECK(strstr(burst, "\nmodulations_list:USB,LSB,CW,CWR,DIGU;\n") != NULL,
        "modulations_list: USB, LSB, CW, CWR, DIGU");
  CHECK(strstr(burst, "\nvfo_limits:500000,32000000;\n") != NULL, "vfo_limits 0.5-32 MHz");
  CHECK(strstr(burst, "\ndds:0,14074000;\n") && strstr(burst, "\nvfo:0,0,14074000;\n"),
        "dds and vfo report the dial");
  CHECK(strstr(burst, "\nmodulation:0,usb;\n") != NULL, "modulation reports the mode");
  CHECK(strstr(burst, "\nvolume:-20;\n") != NULL, "volume 60%% reads as -20 dB");
  CHECK(strstr(burst, "\ndrive:0,40;\n") && strstr(burst, "\ncw_macros_speed:12;\n"),
        "drive and CW speed");
  CHECK(tci_client_count() == 1, "one client counted");
  close(fd);
  long long end = now_ms() + 1000;
  while (tci_client_count() != 0 && now_ms() < end)
    usleep(5000);
  CHECK(tci_client_count() == 0, "and none once it has gone");
}

static void test_sets_and_echoes(void) {
  printf("sets, echoes and reads\n");
  int a = quick_connect(), b = quick_connect();
  CHECK(a >= 0 && b >= 0, "two clients");
  if (a < 0 || b < 0)
    return;
  drain(a, 100);
  drain(b, 100);

  send_text(a, "vfo:0,0,7074000;");
  CHECK(expect(a, "vfo:0,0,7074000;", 500), "vfo set echoed to the setter");
  CHECK(expect(b, "vfo:0,0,7074000;", 500), "and to the other client");
  CHECK(radio_now().freq == 7074000, "the radio was tuned");
  drain(a, 100);
  drain(b, 100);

  send_text(a, "vfo:0,0,100;");
  CHECK(expect(a, "vfo:0,0,7074000;", 500), "out of range: echoed with the frequency in force");
  CHECK(radio_now().freq == 7074000, "and not tuned");
  drain(a, 100);
  drain(b, 100);

  send_text(a, "dds:0;");
  CHECK(expect(a, "dds:0,7074000;", 500), "a read is answered");
  char other[4096];
  CHECK(collect(b, 200, other, sizeof(other)) == 0, "to the asker alone");

  send_text(b, "MODULATION:0,CW;");
  CHECK(expect(b, "modulation:0,cw;", 500), "uppercase set, lowercase echo");
  CHECK(expect(a, "modulation:0,cw;", 500), "mode echoed to every client");
  CHECK(radio_now().mode == TCI_MODE_CW, "mode set");
  send_text(a, "modulation:0,digu;");
  CHECK(expect(a, "modulation:0,digu;", 500) && radio_now().mode == TCI_MODE_DIGU, "digu");
  send_text(a, "modulation:0,am;");
  CHECK(expect(a, "modulation:0,digu;", 500) && radio_now().mode == TCI_MODE_DIGU,
        "a mode not offered: echoed with the mode in force");

  send_text(a, "drive:0,0;");
  CHECK(expect(a, "drive:0,1;", 500) && radio_now().drive == 1, "drive 0 is held at 1");
  send_text(a, "drive:0,55;");
  CHECK(expect(b, "drive:0,55;", 500) && radio_now().drive == 55, "drive 55");

  send_text(a, "volume:-10;");
  CHECK(expect(b, "volume:-10;", 500) && radio_now().volume == 80, "volume -10 dB is 80%%");
  send_text(a, "volume:-60;");
  CHECK(expect(a, "volume:-60;", 500) && radio_now().volume == 0, "volume -60 dB is 0");

  send_text(a, "mute:true;");
  CHECK(expect(b, "mute:true;", 500) && expect(b, "rx_mute:0,true;", 500) && radio_now().mute,
        "mute, echoed as mute and rx_mute");
  send_text(a, "rx_mute:0,false;");
  CHECK(expect(a, "mute:false;", 500) && !radio_now().mute, "rx_mute unmutes");

  send_text(a, "cw_macros_speed:25;");
  CHECK(expect(b, "cw_macros_speed:25;", 500) && expect(b, "cw_keyer_speed:25;", 500) &&
            radio_now().wpm == 25,
        "CW speed, echoed as both speeds");
  send_text(a, "cw_macros_speed_up:3;");
  CHECK(expect(a, "cw_macros_speed:28;", 500) && radio_now().wpm == 28, "speed up 3");
  drain(a, 100);
  drain(b, 100);

  int calls = set_calls;
  send_text(a, "rit_offset:0,100;");
  CHECK(expect(a, "rit_offset:0,0;", 500), "RIT isn't set: echoed with the offset in force");
  send_text(a, "split_enable:true;");
  CHECK(expect(a, "split_enable:0,false;", 500), "split stays off");
  send_text(a, "xit_enable:0,true;");
  CHECK(expect(a, "xit_enable:0,false;", 500), "XIT stays off");
  CHECK(set_calls == calls, "none of them set anything");
  drain(a, 100);
  drain(b, 100);

  send_text(a, "dds:0;modulation:0;drive:0;");
  char got[4096];
  collect(a, 300, got, sizeof(got));
  CHECK(!strcmp(got, "dds:0,7074000;\nmodulation:0,digu;\ndrive:0,55;\n"),
        "three commands in a message, three answers");

  close(a);
  close(b);
}

static void test_dds_and_if(void) {
  printf("dds and if, as a panadapter tunes\n");
  int a = quick_connect(), b = quick_connect();
  if (a < 0 || b < 0) {
    CHECK(0, "two clients");
    return;
  }
  send_text(a, "vfo:0,0,7030000;");
  CHECK(expect(a, "dds:0,7030000;", 500) && expect(a, "if:0,0,0;", 500) &&
            tci_iq_center() == 7030000,
        "a vfo set centres the I/Q on it, if 0");
  drain(a, 100);
  drain(b, 100);

  send_text(a, "if:0,0,1500;");
  CHECK(expect(a, "if:0,0,1500;", 500) && expect(b, "vfo:0,0,7031500;", 500),
        "if:0,0,1500: echoed, and the VFO is dds + if");
  CHECK(radio_now().freq == 7031500 && tci_iq_center() == 7030000,
        "the radio tunes there, the I/Q's centre stays");
  send_text(a, "dds:0;");
  CHECK(expect(a, "dds:0,7030000;", 500), "dds reads the centre");
  send_text(a, "if:0,0;");
  CHECK(expect(a, "if:0,0,1500;", 500), "if reads the offset");
  drain(a, 100);
  drain(b, 100);

  send_text(a, "dds:0,7040000;");
  CHECK(expect(a, "dds:0,7040000;", 500) && expect(a, "vfo:0,0,7041500;", 500) &&
            radio_now().freq == 7041500 && tci_iq_center() == 7040000,
        "a dds set moves the centre and the VFO with it, the offset kept");
  send_text(a, "if:0,0,-800;");
  CHECK(expect(a, "vfo:0,0,7039200;", 500) && radio_now().freq == 7039200,
        "a negative offset");
  drain(a, 100);
  send_text(a, "if:0,0,60000;");
  CHECK(expect(a, "if:0,0,-800;", 500) && radio_now().freq == 7039200,
        "an offset beyond if_limits: echoed with the one in force");
  drain(a, 100);
  drain(b, 100);

  pthread_mutex_lock(&radio_lock);
  radio.freq = 7055000; // the knob
  pthread_mutex_unlock(&radio_lock);
  CHECK(expect(a, "dds:0,7055000;", 500) && expect(a, "if:0,0,0;", 500) &&
            expect(b, "vfo:0,0,7055000;", 500) && tci_iq_center() == 7055000,
        "the dial moved elsewhere: re-centred on it, if 0");
  drain(a, 100);
  char got[2048];
  CHECK(collect(a, 200, got, sizeof(got)) == 0, "and nothing more");
  close(a);
  close(b);
}

static void test_changes_elsewhere(void) {
  printf("changes made elsewhere\n");
  int a = quick_connect(), b = quick_connect();
  if (a < 0 || b < 0) {
    CHECK(0, "two clients");
    return;
  }
  drain(a, 100);
  drain(b, 100);
  pthread_mutex_lock(&radio_lock);
  radio.freq = 10136000;
  radio.mode = TCI_MODE_USB;
  radio.tx = 1;
  pthread_mutex_unlock(&radio_lock);
  long long t0 = now_ms();
  CHECK(expect(a, "dds:0,10136000;", 500) && expect(b, "vfo:0,0,10136000;", 500),
        "a new frequency reaches both clients (%lld ms)", now_ms() - t0);
  CHECK(expect(a, "modulation:0,usb;", 500) && expect(b, "modulation:0,usb;", 500), "and the mode");
  CHECK(expect(a, "trx:0,true;", 500), "transmitting, from any source, is reported");
  pthread_mutex_lock(&radio_lock);
  radio.tx = 0;
  pthread_mutex_unlock(&radio_lock);
  CHECK(expect(a, "trx:0,false;", 500), "and receiving again");
  drain(a, 100);
  char got[4096];
  CHECK(collect(a, 200, got, sizeof(got)) == 0, "nothing more while nothing changes");
  close(a);
  close(b);
}

// Waits up to ms for cond to become true.
#define WAIT_FOR(cond, ms)                                                                         \
  ({                                                                                               \
    long long end_ = now_ms() + (ms);                                                              \
    while (!(cond) && now_ms() < end_)                                                             \
      usleep(5000);                                                                                \
    (cond);                                                                                        \
  })

// A client's TX audio frame: frames stereo float32 frames, left[k] = f(k).
static void send_tx_audio(int fd, int frames, float (*f)(int)) {
  size_t len = TCI_HEADER_BYTES + (size_t)frames * 8;
  uint8_t *b = calloc(1, len);
  uint32_t hdr[8] = {0, 48000, TCI_FLOAT32, 0, 0, (uint32_t)(2 * frames), TCI_TX_AUDIO_STREAM, 2};
  for (int k = 0; k < 8; k++)
    for (int j = 0; j < 4; j++)
      b[4 * k + j] = (uint8_t)(hdr[k] >> (8 * j));
  for (int k = 0; k < frames; k++)
    memcpy(b + TCI_HEADER_BYTES + 8 * k, &(float){f(k)}, 4);
  send_frame(fd, 1, 2, b, len, 1);
  free(b);
}

static float ramp(int k) { return (float)(k - 512) / 1024.0f; }

static void test_tx(void) {
  printf("transmit\n");
  usleep(200000); // the earlier clients have gone
  int a = quick_connect(), b = quick_connect();
  if (a < 0 || b < 0) {
    CHECK(0, "two clients");
    return;
  }
  drain(a, 100);
  drain(b, 100);

  send_text(a, "tx_stream_audio_buffering:50;");
  CHECK(expect(a, "tx_stream_audio_buffering:50;", 500), "buffering set");
  send_text(a, "trx:0,true,tci;");
  CHECK(expect(a, "trx:0,true;", 500) && expect(b, "trx:0,true;", 500),
        "trx:0,true,tci echoed to every client");
  CHECK(radio_now().tx && tx_own_audio && tci_holds_tx() && tci_tx_audio_owned(),
        "keyed, with the client's own audio");

  size_t len = 0;
  uint8_t *f = next_stream(a, TCI_TX_CHRONO, 500, &len);
  CHECK(f && get_u32(f + 4) == 48000 && get_u32(f + 20) == 2048 && get_u32(f + 28) == 2,
        "TX_CHRONO asks for 1024 stereo frames at 48 kHz");
  free(f);
  uint8_t *none = next_stream(b, TCI_TX_CHRONO, 200, &len);
  CHECK(!none, "only of the client that holds TX");
  free(none);

  double out[1024];
  while (tci_pull_audio_tx(out, 1024) > 0)
    ; // nothing queued yet
  send_tx_audio(a, 1024, ramp);
  int got = 0;
  WAIT_FOR((got += tci_pull_audio_tx(out + got, 1024 - got)) >= 1024, 500);
  int same = got == 1024;
  for (int k = 0; same && k < 1024; k++)
    same = out[k] == ramp(k);
  CHECK(same, "its audio reaches the audio thread, left channel, in order (%d samples)", got);
  send_tx_audio(b, 1024, ramp);
  usleep(100000);
  CHECK(tci_pull_audio_tx(out, 1024) == 0, "another client's audio is dropped");

  int calls = tx_calls_now();
  send_text(b, "trx:0,false;");
  CHECK(expect(b, "trx:0,true;", 500) && radio_now().tx && tx_calls_now() == calls,
        "another client can't unkey it");
  send_text(b, "trx:0,true,tci;");
  CHECK(expect(b, "trx:0,true;", 500) && tx_calls_now() == calls, "nor take it");
  send_text(a, "trx:0,true,tci;");
  CHECK(expect(a, "trx:0,true;", 500) && tx_calls_now() == calls, "a repeated trx changes nothing");

  send_text(a, "trx:0,false;");
  CHECK(expect(a, "trx:0,false;", 500) && expect(b, "trx:0,false;", 500), "trx:0,false echoed");
  CHECK(!radio_now().tx && !tci_holds_tx() && !tci_tx_audio_owned(), "unkeyed and released");
  send_tx_audio(a, 1024, ramp);
  usleep(100000);
  CHECK(tci_pull_audio_tx(out, 1024) == 0, "audio after release is dropped");

  send_text(b, "trx:0,true;");
  CHECK(expect(b, "trx:0,true;", 500) && radio_now().tx && !tx_own_audio && !tci_tx_audio_owned(),
        "trx with no source: keyed with sbitx's audio");
  send_text(b, "trx:0,false;");
  CHECK(expect(b, "trx:0,false;", 500) && !tci_holds_tx(), "and released");

  pthread_mutex_lock(&radio_lock);
  radio.tx = 1; // the microphone's PTT, say
  pthread_mutex_unlock(&radio_lock);
  drain(a, 200);
  calls = tx_calls_now();
  send_text(a, "trx:0,true,tci;");
  CHECK(expect(a, "trx:0,true;", 500) && tx_calls_now() == calls && !tci_holds_tx(),
        "while sbitx transmits for something else, trx is ignored");
  send_text(a, "trx:0,false;");
  CHECK(expect(a, "trx:0,true;", 500) && radio_now().tx, "and can't unkey it");
  pthread_mutex_lock(&radio_lock);
  radio.tx = 0;
  pthread_mutex_unlock(&radio_lock);
  drain(a, 200);
  drain(b, 200);

  tx_refuse = 1; // sbitx's mode doesn't take it
  send_text(b, "trx:0,true,tci;");
  CHECK(expect(b, "trx:0,false;", 500) && !tci_holds_tx() && !tci_tx_audio_owned() &&
            !radio_now().tx,
        "a trx the radio refuses: answered false, nothing held");
  tx_refuse = 0;
  drain(a, 200);
  drain(b, 200);

  send_text(b, "trx:0,true,tci;");
  expect(b, "trx:0,true;", 500);
  pthread_mutex_lock(&radio_lock);
  radio.tx = 0; // sbitx ended it: a timeout, the VSWR trip
  pthread_mutex_unlock(&radio_lock);
  CHECK(WAIT_FOR(!tci_holds_tx(), 500) && expect(a, "trx:0,false;", 500),
        "TX ending in sbitx releases the client, and is reported");

  send_text(a, "trx:0,true,tci;");
  expect(a, "trx:0,true;", 500);
  close(a);
  CHECK(WAIT_FOR(!tci_holds_tx() && !radio_now().tx, 1500),
        "a client that disconnects while holding TX unkeys it");

  send_text(b, "trx:0,true,tci;");
  expect(b, "trx:0,true;", 500);
  tci_stop();
  CHECK(!tci_holds_tx() && !radio_now().tx, "stopping the server unkeys it");
  close(b);
  tci_init("127.0.0.1", ws_test_port, TCI_DEFAULT_MAX_CLIENTS);
}

static void test_limit(void) {
  printf("client limit\n");
  int fds[TCI_DEFAULT_MAX_CLIENTS];
  int all = 1;
  usleep(200000); // the earlier clients' slots
  for (int k = 0; k < TCI_DEFAULT_MAX_CLIENTS; k++)
    if ((fds[k] = quick_connect()) < 0)
      all = 0;
  CHECK(all, "%d clients at once", TCI_DEFAULT_MAX_CLIENTS);
  int extra = ws_connect();
  CHECK(extra < 0, "one more is refused");
  if (extra >= 0)
    close(extra);
  for (int k = 0; k < TCI_DEFAULT_MAX_CLIENTS; k++)
    if (fds[k] >= 0)
      close(fds[k]);
  usleep(200000);
}

static void test_cw_text(void) {
  printf("CW text\n");
  int fd = quick_connect();
  if (fd < 0) {
    CHECK(0, "a client");
    return;
  }
  drain(fd, 100);
  static const struct {
    const char *sent, *text;
  } cases[] = {{"cw_macros:0,CQ TEST DE KB2ML;", "CQ TEST DE KB2ML"},
               {"CW_MACROS:0,|SK| <5NN>;", "|SK| <5NN>"},
               {"cw_macros:0, TU  ;", "TU"},
               {"cw_macros:0,5NN, TU;", "5NN,TU"},
               {"cw_macros:0,a^b~c*d;", "a^b~c*d"}};
  for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
    pthread_mutex_lock(&radio_lock);
    int before = cw_texts;
    pthread_mutex_unlock(&radio_lock);
    send_text(fd, cases[k].sent);
    long long end = now_ms() + 500;
    int got = 0;
    while (!got && now_ms() < end) {
      pthread_mutex_lock(&radio_lock);
      got = cw_texts > before && !strcmp(cw_text, cases[k].text);
      pthread_mutex_unlock(&radio_lock);
      usleep(5000);
    }
    CHECK(got, "%s gives the text \"%s\"", cases[k].sent, cases[k].text);
  }
  pthread_mutex_lock(&radio_lock);
  int before = cw_texts;
  pthread_mutex_unlock(&radio_lock);
  send_text(fd, "cw_macros:1,X;");
  send_text(fd, "cw_macros:0;");
  usleep(200000);
  pthread_mutex_lock(&radio_lock);
  CHECK(cw_texts == before, "another transceiver's, or none, is ignored");
  int stops = cw_stops;
  pthread_mutex_unlock(&radio_lock);
  send_text(fd, "cw_macros_stop;");
  usleep(200000);
  pthread_mutex_lock(&radio_lock);
  CHECK(cw_stops == stops + 1, "cw_macros_stop stops it");
  pthread_mutex_unlock(&radio_lock);
  char got[512];
  CHECK(collect(fd, 200, got, sizeof(got)) == 0, "none of them is answered");
  close(fd);
}

static void test_streams(void) {
  printf("receive streams\n");
  int fd = quick_connect();
  if (fd < 0) {
    CHECK(0, "a client");
    return;
  }
  send_text(fd, "audio_stream_sample_type:float32;");
  send_text(fd, "audio_stream_channels:1;");
  send_text(fd, "audio_start:0;");
  CHECK(expect(fd, "audio_start:0;", 500), "audio_start answered");

  // 4096 samples at 48 kHz: two frames of 2048
  static double audio[4096];
  for (int k = 0; k < 4096; k++)
    audio[k] = (k - 2048) / 4096.0;
  tci_push_audio_rx(audio, 4096);
  size_t len = 0;
  uint8_t *f = next_stream(fd, TCI_RX_AUDIO_STREAM, 1000, &len);
  CHECK(f != NULL, "a receive audio frame");
  if (f) {
    uint32_t n = get_u32(f + 20);
    CHECK(get_u32(f + 4) == 48000 && get_u32(f + 8) == TCI_FLOAT32 && get_u32(f + 28) == 1,
          "48 kHz, float32, mono");
    CHECK(n == 2048 && len == TCI_HEADER_BYTES + 4 * n, "2048 samples");
    int same = 1;
    for (uint32_t k = 0; k < n && k < len / 4; k++)
      if (get_f32(f + TCI_HEADER_BYTES + 4 * k) != (float)audio[k])
        same = 0;
    CHECK(same, "the samples as given, in order");
    free(f);
  }
  send_text(fd, "audio_stop:0;");
  CHECK(expect(fd, "audio_stop:0;", 500), "audio_stop answered");

  send_text(fd, "iq_samplerate:96000;");
  send_text(fd, "iq_start:0;");
  CHECK(expect(fd, "iq_start:0;", 500), "iq_start answered");
  static double i_in[2048], q_in[2048];
  for (int k = 0; k < 2048; k++) {
    i_in[k] = cos(2 * M_PI * 1000 * k / 96000.0) * 0.5;
    q_in[k] = sin(2 * M_PI * 1000 * k / 96000.0) * 0.5; // a station 1 kHz above the dial
  }
  tci_push_iq(i_in, q_in, 2048);
  f = next_stream(fd, TCI_IQ_STREAM, 1000, &len);
  CHECK(f != NULL, "an I/Q frame");
  if (f) {
    uint32_t n = get_u32(f + 20);
    CHECK(get_u32(f + 4) == 96000 && get_u32(f + 8) == TCI_FLOAT32 && get_u32(f + 28) == 2,
          "96 kHz, float32, two channels");
    CHECK(n == 4096 && len == TCI_HEADER_BYTES + 4 * n, "2048 complex samples");
    int same = 1;
    for (uint32_t k = 0; k < 2048 && 8 * k + 4 < len; k++)
      if (get_f32(f + TCI_HEADER_BYTES + 8 * k) != (float)i_in[k] ||
          get_f32(f + TCI_HEADER_BYTES + 8 * k + 4) != (float)q_in[k])
        same = 0;
    CHECK(same, "I then Q, as given: a station above the dial stays above it");
    free(f);
  }
  send_text(fd, "iq_stop:0;");
  CHECK(expect(fd, "iq_stop:0;", 500), "iq_stop answered");
  close(fd);
}

static void test_restart(int port) {
  printf("stop and start\n");
  tci_stop();
  CHECK(!tci_running(), "stopped");
  int fd = ws_connect();
  CHECK(fd < 0, "nothing listens");
  if (fd >= 0)
    close(fd);
  CHECK(tci_init("127.0.0.1", port, TCI_DEFAULT_MAX_CLIENTS) == 0 && tci_running(),
        "started again on the same port");
  CHECK(tci_init("127.0.0.1", port, TCI_DEFAULT_MAX_CLIENTS) == 0, "starting again is harmless");
  char burst[8192];
  fd = connect_client(burst, sizeof(burst), NULL, NULL);
  CHECK(fd >= 0 && strstr(burst, "\nready;\n"), "a client gets the burst");
  if (fd >= 0)
    close(fd);
}

static double tone_gain(double hz) {
  struct decim48k_state d = {0};
  double peak = 0, y;
  for (int k = 0; k < 9600; k++)
    if (decim48k_apply(&d, sin(2 * M_PI * hz * k / 96000.0), &y) && k > 200 && fabs(y) > peak)
      peak = fabs(y);
  return 20 * log10(peak + 1e-12);
}

static void test_decim48k(void) {
  printf("decim48k\n");
  double g1 = tone_gain(1000), g3 = tone_gain(3000), g15 = tone_gain(15000);
  CHECK(fabs(g1) < 0.3 && fabs(g3) < 0.3 && fabs(g15) < 0.3,
        "flat to 15 kHz (%.2f, %.2f, %.2f dB at 1, 3, 15 kHz)", g1, g3, g15);
  double g30 = tone_gain(30000), g40 = tone_gain(40000);
  CHECK(g30 < -50 && g40 < -50, "what would alias is down %.0f and %.0f dB at 30 and 40 kHz",
        -g30, -g40);
}

// The levels of the hz component and of its image at 48000 - hz in the
// 96 kHz output, for a 48 kHz tone at hz.
static void upsample_levels(double hz, double *wanted_db, double *image_db) {
  struct upsample48k_state u = {0};
  enum { N = 9600 };
  static double y[2 * N];
  for (int k = 0; k < N; k++)
    upsample48k_apply(&u, sin(2 * M_PI * hz * k / 48000.0), y + 2 * k);
  double fw[2] = {hz, 48000.0 - hz}, lvl[2];
  for (int w = 0; w < 2; w++) {
    double re = 0, im = 0;
    for (int k = 400; k < 2 * N; k++) {
      re += y[k] * cos(2 * M_PI * fw[w] * k / 96000.0);
      im += y[k] * sin(2 * M_PI * fw[w] * k / 96000.0);
    }
    lvl[w] = 20 * log10(2 * sqrt(re * re + im * im) / (2 * N - 400) + 1e-12);
  }
  *wanted_db = lvl[0];
  *image_db = lvl[1];
}

static void test_upsample48k(void) {
  printf("upsample48k\n");
  double w1, i1, w3, i3;
  upsample_levels(1000, &w1, &i1);
  upsample_levels(3000, &w3, &i3);
  CHECK(fabs(w1) < 0.3 && fabs(w3) < 0.3, "unity gain (%.2f dB at 1 kHz, %.2f dB at 3 kHz)", w1, w3);
  CHECK(i1 < -50 && i3 < -50, "the images at 47 and 45 kHz down %.0f and %.0f dB", -i1, -i3);
}

int main(void) {
  for (ws_test_port = 45951; ws_test_port < 46000; ws_test_port++)
    if (tci_init("127.0.0.1", ws_test_port, TCI_DEFAULT_MAX_CLIENTS) == 0)
      break;
  printf("tci bench harness, loopback port %d\n", ws_test_port);

  test_burst();
  test_sets_and_echoes();
  test_changes_elsewhere();
  test_dds_and_if();
  test_tx();
  test_cw_text();
  test_limit();
  test_streams();
  test_restart(ws_test_port);
  test_decim48k();
  test_upsample48k();

  tci_stop();
  printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
