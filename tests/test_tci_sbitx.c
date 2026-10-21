// test_tci_sbitx.c
//
// Bench harness for tci_sbitx.c, sbitx's side of TCI, with sbitx's fields
// and command queue stubbed: the TCI setting starting the server, the
// state published from the fields, the commands a client's trx becomes
// (t, rtx, r) or its refusal in each mode, the TX state reported until
// sbitx shows it, and the transmit audio block - which modes take a
// client's audio, its level (full scale is sbitx's own FT8 tones' level), clipping, the 48 to 96 kHz upsampling,
// silence when the client is late, and silence rather than the microphone
// between a client's unkeying and sbitx receiving again; and CW text -
// TCI's markup translated for sbitx's keyer, and the queue into the TEXT
// field: CW and CWR only, in order past the field's length, stopped by
// cw_macros_stop or the key, dropped on leaving CW. The harness plays the GTK and audio threads itself.
// Built and run by make test, as /tmp/sbitx-test_tci_sbitx.

#define _GNU_SOURCE
#include "sdr.h"
#include "tci.h"
#include "tci_radio.h"
#include "tci_sbitx.h"
#include "tci_cw.h"
#include "tci_stream.h"
#include "ws_test_client.h"
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

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

/* ---- sbitx, stubbed ------------------------------------------------------- */

static struct {
  const char *label;
  char value[32];
} fields[] = {{"FREQ", "7074000"}, {"MODE", "DIGI"}, {"DRIVE", "40"},  {"AUDIO", "60"},
              {"MUTE", "OFF"},     {"RIT", "OFF"},   {"RIT_DELTA", "0"}, {"WPM", "12"},
              {"TCI", "OFF"},      {"TCI_PORT", "46071"}};

static char *field(const char *label) {
  for (size_t k = 0; k < sizeof(fields) / sizeof(fields[0]); k++)
    if (!strcmp(fields[k].label, label))
      return fields[k].value;
  return NULL;
}
const char *field_str(const char *label) { return field(label); }
int field_int(char *label) {
  const char *v = field(label);
  return v ? atoi(v) : 0;
}
static void set(const char *label, const char *value) { snprintf(field(label), 32, "%s", value); }

int in_tx, audio_muted;
char premute_volume[16];
struct rx *rx_list;
int calculate_s_meter(struct rx *r, double rx_gain) {
  (void)r;
  (void)rx_gain;
  return 900;
}
int get_rx_gain(void) { return 100; }

static char last_cmd[64];
static int cmds;
void remote_execute(char *cmd) {
  snprintf(last_cmd, sizeof(last_cmd), "%s", cmd);
  cmds++;
}

// The keyer's queue, the TEXT field, and the key
static char text_field[128];
static int text_clears;
static unsigned key_presses;
int cw_text_append(const char *text) {
  int len = (int)strlen(text_field), room = (int)sizeof(text_field) - 1 - len;
  int n = (int)strlen(text);
  if (n > room)
    n = room;
  memcpy(text_field + len, text, (size_t)n);
  text_field[len + n] = '\0';
  return n;
}
void cw_text_clear(void) {
  text_field[0] = '\0';
  text_clears++;
}
unsigned cw_key_press_count(void) { return key_presses; }
int get_tx_data_length(void) { return (int)strlen(text_field); }

// The keyer sends n characters
static void keyer_takes(int n) {
  int len = (int)strlen(text_field);
  if (n > len)
    n = len;
  memmove(text_field, text_field + n, (size_t)(len - n) + 1);
}

static void ticks(int n) {
  for (int k = 0; k < n; k++)
    tci_sbitx_tick();
}

static struct tci_radio_state state(void) {
  struct tci_radio_state s;
  tci_radio_get(&s);
  return s;
}

/* ---- Tests ---------------------------------------------------------------- */

// The frequency of the I/Q frames a client gets, from their phase steps.
static double iq_frequency(int fd, int frames) {
  double re = 0, im = 0, pi_ = 0, pq_ = 0;
  int have_prev = 0;
  for (int f = 0; f < frames; f++) {
    uint8_t *d = NULL;
    size_t len;
    int op;
    do {
      free(d);
      d = NULL;
      struct pollfd p = {fd, POLLIN, 0};
      if (poll(&p, 1, 1000) <= 0)
        return NAN;
      op = recv_frame(fd, &d, &len, NULL);
    } while (op == 1 || (op == 2 && (len < TCI_HEADER_BYTES || d[24] != TCI_IQ_STREAM)));
    if (op != 2) {
      free(d);
      return NAN;
    }
    for (size_t k = TCI_HEADER_BYTES; k + 8 <= len; k += 8) {
      float i, q;
      memcpy(&i, d + k, 4);
      memcpy(&q, d + k + 4, 4);
      if (have_prev) { // z[k] * conj(z[k-1])
        re += i * pi_ + q * pq_;
        im += q * pi_ - i * pq_;
      }
      pi_ = i;
      pq_ = q;
      have_prev = 1;
    }
    free(d);
  }
  return atan2(im, re) * 96000.0 / (2 * M_PI);
}

// The audio thread: blocks of a tone at hz in the I/Q, shifted by shift.
static void iq_tone_blocks(double hz, int shift, int blocks) {
  static double i[1024], q[1024];
  static long t;
  for (int b = 0; b < blocks; b++) {
    for (int k = 0; k < 1024; k++, t++) {
      i[k] = 0.5 * cos(2 * M_PI * hz * t / 96000.0);
      q[k] = 0.5 * sin(2 * M_PI * hz * t / 96000.0);
    }
    tci_sbitx_rx_block(NULL, i, q, 1024, shift);
    usleep(2000);
  }
}

// Discards what the client has been sent, then the I/Q's frequency for a
// 1000 Hz tone shifted by shift.
static double shifted(int fd, int shift) {
  usleep(100000);
  uint8_t *d = NULL;
  size_t len;
  struct pollfd p = {fd, POLLIN, 0};
  while (poll(&p, 1, 50) > 0 && recv_frame(fd, &d, &len, NULL) >= 0) {
    free(d);
    d = NULL;
  }
  iq_tone_blocks(1000, shift, 10);
  return iq_frequency(fd, 3);
}

static void test_iq_shift(void) {
  printf("the I/Q's shift\n");
  ws_test_port = 46071;
  int fd = ws_connect();
  if (fd < 0) {
    CHECK(0, "a client");
    return;
  }
  const char *start = "iq_samplerate:96000;iq_start:0;";
  send_frame(fd, 1, 1, start, strlen(start), 1);
  double f0 = shifted(fd, 0), f1 = shifted(fd, -700), f2 = shifted(fd, 20000);
  CHECK(fabs(f0 - 1000) < 1, "no shift: a 1000 Hz tone stays at %.1f Hz", f0);
  CHECK(fabs(f1 - 300) < 1, "shifted -700 Hz (CW's pitch): %.1f Hz", f1);
  CHECK(fabs(f2 - 21000) < 1, "shifted +20 kHz (an if offset): %.1f Hz", f2);
  usleep(100000);
  { // beyond the I/Q's width: silence
    uint8_t *d = NULL;
    size_t len;
    struct pollfd p = {fd, POLLIN, 0};
    while (poll(&p, 1, 50) > 0 && recv_frame(fd, &d, &len, NULL) >= 0) {
      free(d);
      d = NULL;
    }
    iq_tone_blocks(1000, 60000, 10);
    double peak = 0;
    for (int f = 0; f < 3 && poll(&p, 1, 1000) > 0; f++) {
      if (recv_frame(fd, &d, &len, NULL) == 2 && len > TCI_HEADER_BYTES && d[24] == TCI_IQ_STREAM)
        for (size_t k = TCI_HEADER_BYTES; k + 4 <= len; k += 4) {
          float x;
          memcpy(&x, d + k, 4);
          peak = fabs(x) > peak ? fabs(x) : peak;
        }
      free(d);
      d = NULL;
    }
    CHECK(peak == 0, "a shift beyond the I/Q's width sends silence");
  }
  close(fd);
}

static void test_setting_and_state(void) {
  printf("the TCI setting and the published state\n");
  ticks(200);
  CHECK(!tci_running(), "off while TCI is OFF");
  set("TCI", "ON");
  ticks(100);
  CHECK(tci_running(), "on within 100 ticks of TCI ON");
  ticks(10);
  struct tci_radio_state s = state();
  CHECK(s.freq == 7074000 && s.mode == TCI_MODE_DIGU && s.drive == 40 && s.volume == 60 &&
            s.wpm == 12 && !s.tx,
        "the fields, published");
  set("MODE", "FT8");
  ticks(10);
  CHECK(state().mode == TCI_MODE_DIGU, "FT8 reads as digu");
  set("MODE", "DIGI");
  ticks(10);
}

static void test_trx_commands(void) {
  printf("what a trx becomes\n");
  static const struct {
    const char *mode;
    int own_audio;
    const char *cmd;
  } cases[] = {{"DIGI", 1, "t"},  {"DIGI", 0, "rtx"}, {"FT8", 0, "rtx"}, {"FT4", 0, "rtx"},
               {"FT8", 1, "t"},   {"FT4", 1, "t"},    {"USB", 0, "t"},   {"USB", 1, "t"},
               {"LSB", 1, "t"},   {"AM", 0, "t"},     {"FM", 0, "t"},    {"CW", 0, "t"},
               {"AM", 1, NULL},   {"FM", 1, NULL},    {"CW", 1, NULL},   {"CWR", 1, NULL},
               {"2TONE", 0, NULL}, {"2TONE", 1, NULL}};
  for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
    set("MODE", cases[k].mode);
    ticks(10);
    int before = cmds;
    int r = tci_radio_set_tx(1, cases[k].own_audio);
    if (!cases[k].cmd) {
      CHECK(r < 0 && cmds == before && !state().tx, "%s, %s audio: refused", cases[k].mode,
            cases[k].own_audio ? "its own" : "sbitx's");
      continue;
    }
    CHECK(r == 0 && !strcmp(last_cmd, cases[k].cmd), "%s, %s audio: %s", cases[k].mode,
          cases[k].own_audio ? "its own" : "sbitx's", last_cmd);
    tci_radio_set_tx(0, 0);
    CHECK(!strcmp(last_cmd, "r"), "  then r");
    ticks(1);
  }
  set("MODE", "DIGI");
  ticks(10);

  tci_radio_set_tx(1, 1);
  CHECK(state().tx, "TX reported at once, before sbitx shows it");
  in_tx = 1;
  ticks(10);
  CHECK(state().tx, "and while it does");
  tci_radio_set_tx(0, 0);
  CHECK(!state().tx, "TX off reported at once");
  ticks(10);
  CHECK(!state().tx, "and held off while sbitx catches up");
  in_tx = 0;
  ticks(10);
  CHECK(!state().tx, "and off");

  tci_radio_set_tx(1, 1); // sbitx doesn't key
  usleep(1100000);
  ticks(10);
  CHECK(!state().tx, "TX sbitx didn't take is reported off after a second");
}

// A client's TX audio, as tci_stream.c receives it: a 48 kHz tone.
static void client_audio(double amplitude, double hz, int frames) {
  static int phase;
  size_t len = TCI_HEADER_BYTES + (size_t)frames * 8;
  uint8_t *b = calloc(1, len);
  uint32_t hdr[8] = {0, 48000, TCI_FLOAT32, 0, 0, (uint32_t)(2 * frames), TCI_TX_AUDIO_STREAM, 2};
  memcpy(b, hdr, sizeof(hdr)); // little-endian, as the Pi and a PC are
  for (int k = 0; k < frames; k++, phase++) {
    float x = (float)(amplitude * sin(2 * M_PI * hz * phase / 48000.0));
    memcpy(b + TCI_HEADER_BYTES + 8 * k, &x, 4);
  }
  tci_stream_tx_frame(0, b, len);
  free(b);
}

static void test_tx_audio(void) {
  printf("transmit audio\n");
  static int32_t mic[1024];
  for (int k = 0; k < 1024; k++)
    mic[k] = 12345;
  CHECK(!tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL) && mic[0] == 12345,
        "no client's audio: the microphone or loopback stays");

  tci_stream_tx_begin(0, 50);
  static const struct {
    int mode;
    const char *name;
    int takes;
  } modes[] = {{MODE_DIGITAL, "DIGI", 1}, {MODE_FT8, "FT8", 1},  {MODE_FT4, "FT4", 1},
               {MODE_USB, "USB", 1},      {MODE_LSB, "LSB", 1},  {MODE_AM, "AM", 0},
               {MODE_FM, "FM", 0},        {MODE_CW, "CW", 0},    {MODE_CWR, "CWR", 0},
               {MODE_2TONE, "2TONE", 0}};
  for (size_t k = 0; k < sizeof(modes) / sizeof(modes[0]); k++) {
    client_audio(0.5, 1000, 512);
    for (int j = 0; j < 1024; j++)
      mic[j] = 12345;
    int took = tci_sbitx_tx_block(mic, 1024, modes[k].mode);
    CHECK(took == modes[k].takes && (took || mic[0] == 12345), "%s %s the client's audio",
          modes[k].name, modes[k].takes ? "takes" : "doesn't take");
    tci_tx_audio_idle();
  }

  // the level and the tone, after the filter has settled
  double peak = 0;
  int crossings = 0;
  for (int block = 0; block < 20; block++) {
    client_audio(0.5, 1000, 512);
    tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL);
    if (block < 2)
      continue;
    for (int j = 0; j < 1024; j++) {
      if (fabs((double)mic[j]) > peak)
        peak = fabs((double)mic[j]);
      if (j > 0 && (mic[j - 1] < 0) != (mic[j] < 0))
        crossings++;
    }
  }
  // sbitx's own FT8 tones reach tx_process() at 1/21 of full scale, and
  // tx_process() divides input_mic by 2e9: a program's full scale is that
  double db = 20 * log10(peak / (0.5 * 2.0e9 / 21.0));
  CHECK(fabs(db) < 0.3, "0.5 is half the level of sbitx's own FT8 tones (%.2f dB)", db);
  double hz = crossings / 2.0 / (18 * 1024 / 96000.0);
  CHECK(fabs(hz - 1000) < 5, "a 1 kHz tone stays 1 kHz (%.1f Hz)", hz);

  int32_t lo = 0, hi = 0;
  for (int block = 0; block < 4; block++) {
    client_audio(1.5, 1000, 512);
    tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL);
    for (int j = 0; j < 1024; j++) {
      lo = mic[j] < lo ? mic[j] : lo;
      hi = mic[j] > hi ? mic[j] : hi;
    }
  }
  int32_t top = (int32_t)lrint(2.0e9 / 21.0);
  CHECK(hi == top && lo == -top, "over full scale clips at sbitx's FT8 level");

  int got = tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL); // nothing sent
  int quiet = 1;
  for (int j = 64; j < 1024; j++)
    quiet = quiet && abs(mic[j]) < 1000;
  CHECK(got && quiet, "a late client: silence, not stale audio");

  client_audio(0.5, 1000, 512);
  tci_sbitx_tx_idle(); // receiving, and no client holds TX
  tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL);
  quiet = 1;
  for (int j = 64; j < 1024; j++)
    quiet = quiet && abs(mic[j]) < 1000;
  CHECK(quiet, "audio queued while receiving is discarded");
  tci_stream_tx_end();

  // unkeyed, and sbitx not yet receiving
  set("MODE", "DIGI");
  ticks(10);
  tci_radio_set_tx(1, 1);
  in_tx = 1;
  ticks(10);
  tci_radio_set_tx(0, 0); // tci.c then ends the stream
  for (int j = 0; j < 1024; j++)
    mic[j] = 12345;
  CHECK(tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL) && mic[0] == 0 && mic[1023] == 0,
        "until r runs, an own-audio TX sends silence, not the microphone");
  CHECK(tci_sbitx_tx_engaged(), "and sbitx's FT8 still stands aside");
  ticks(5);
  CHECK(tci_sbitx_tx_engaged(), "while sbitx is still transmitting");
  in_tx = 0; // r has run
  ticks(1);
  mic[0] = 12345;
  CHECK(!tci_sbitx_tx_engaged() && !tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL) &&
            mic[0] == 12345,
        "then neither");
  tci_radio_set_tx(1, 0); // sbitx's audio
  in_tx = 1;
  ticks(10);
  tci_radio_set_tx(0, 0);
  CHECK(!tci_sbitx_tx_block(mic, 1024, MODE_DIGITAL) && mic[0] == 12345,
        "a TX with sbitx's audio keeps it to the end");
  in_tx = 0;
  ticks(1);
}

static void test_translate(void) {
  printf("TCI CW text to sbitx's keyer\n");
  static const struct {
    const char *in, *out;
  } cases[] = {{"CQ TEST DE KB2ML", "CQ TEST DE KB2ML"},
               {"cq de kb2ml k", "cq de kb2ml k"},
               {"|AR| |sk| |BT| |KN| |AS| |BK|", "+ > = ( & <"},
               {"|HH| |XY|", "HH XY"},
               {"TU <5NN> TU", "TU [5NN] TU"},
               {"<<5NN", "[[5NN]]"},
               {">599", "]599["},
               {"a^b~c*d", "a:b,c;d"},
               {"5NN\tTU\n", "5NN TU "},
               {"?./-'!)(\"@$+=&_", "?./-'!)(\"@$+=&_"}};
  for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
    char out[64], skipped[16];
    tci_cw_translate(cases[k].in, out, sizeof(out), skipped, sizeof(skipped));
    CHECK(!strcmp(out, cases[k].out) && !skipped[0], "\"%s\" -> \"%s\"", cases[k].in, out);
  }
  char out[64], skipped[16];
  tci_cw_translate("A[B]C\\D{E}#%|F", out, sizeof(out), skipped, sizeof(skipped));
  CHECK(!strcmp(out, "ABCDEF") && !strcmp(skipped, "[]\\{}#%|"),
        "what sbitx can't send is left out and reported (%s, %s)", out, skipped);
  char small[8];
  tci_cw_translate("<<ABCDEFGH", small, sizeof(small), NULL, 0);
  CHECK(!strcmp(small, "[[ABC]]"), "cut short, it still ends at WPM (%s)", small);
}

static void test_cw_queue(void) {
  printf("CW text into the keyer's queue\n");
  set("MODE", "USB");
  ticks(10);
  tci_radio_cw_text("CQ");
  ticks(5);
  CHECK(!text_field[0], "refused outside CW and CWR");

  set("MODE", "CW");
  ticks(10);
  tci_radio_cw_text("CQ TEST");
  ticks(1);
  CHECK(!strcmp(text_field, "CQ TEST"), "in CW, into the TEXT field (%s)", text_field);
  tci_radio_cw_text("DE KB2ML <5NN>");
  ticks(1);
  CHECK(!strcmp(text_field, "CQ TEST DE KB2ML [5NN]"),
        "the next after it, translated, a word space between (%s)", text_field);
  keyer_takes(100);
  ticks(1);
  tci_radio_cw_text("TU");
  ticks(1);
  CHECK(!strcmp(text_field, "TU"), "no space before one that starts an empty queue (%s)",
        text_field);
  keyer_takes(100);
  ticks(1);

  char longer[400] = "", expect[400] = "";
  for (int k = 0; k < 30; k++)
    strcat(longer, "ABCDEFGHI ");
  strcpy(expect, longer);
  tci_radio_cw_text(longer);
  ticks(1);
  char sent[400] = "";
  CHECK(strlen(text_field) == 127, "a long text fills the field (%zu)", strlen(text_field));
  for (int k = 0; k < 20 && strlen(sent) < strlen(expect); k++) {
    strncat(sent, text_field, 40);
    keyer_takes(40);
    ticks(1);
  }
  CHECK(!strcmp(sent, expect), "and the rest follows in order as the keyer sends it");
  ticks(1);

  int clears = text_clears;
  tci_radio_cw_text("CQ CQ");
  ticks(1);
  tci_radio_cw_stop();
  ticks(1);
  CHECK(!text_field[0] && text_clears == clears + 1, "cw_macros_stop empties the queue");

  clears = text_clears;
  for (int k = 0; k < 10; k++)
    tci_radio_cw_text("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  ticks(1);
  key_presses++; // the operator keys
  ticks(1);
  keyer_takes(200);
  ticks(5);
  CHECK(!text_field[0] && text_clears == clears + 1, "the key closing stops TCI text");
  key_presses++;
  ticks(1);
  CHECK(text_clears == clears + 1, "and keying with no TCI text stops nothing");

  clears = text_clears;
  for (int k = 0; k < 10; k++)
    tci_radio_cw_text("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  ticks(1);
  set("MODE", "USB"); // sbitx's mode change clears the field itself
  text_field[0] = '\0';
  ticks(10);
  set("MODE", "CW");
  ticks(10);
  CHECK(!text_field[0] && text_clears == clears, "leaving CW drops what was waiting");
}

int main(void) {
  tci_stream_init();
  test_setting_and_state();
  test_trx_commands();
  test_tx_audio();
  test_translate();
  test_cw_queue();
  test_iq_shift();
  tci_stop();
  printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
