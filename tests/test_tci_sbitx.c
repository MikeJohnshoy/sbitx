// test_tci_sbitx.c
//
// Bench harness for tci_sbitx.c, sbitx's side of TCI, with sbitx's fields
// and command queue stubbed: the TCI setting starting the server, the
// state published from the fields, the commands a client's trx becomes
// (t, rtx, r) or its refusal in each mode, the TX state reported until
// sbitx shows it, and the transmit audio block - which modes take a
// client's audio, its level (full scale is sbitx's own FT8 tones' level), clipping, the 48 to 96 kHz upsampling,
// silence when the client is late, and silence rather than the microphone
// between a client's unkeying and sbitx receiving again. The harness plays the GTK and audio threads itself.
// Built and run by make test, as /tmp/sbitx-test_tci_sbitx.

#define _GNU_SOURCE
#include "sdr.h"
#include "tci.h"
#include "tci_radio.h"
#include "tci_sbitx.h"
#include "tci_stream.h"
#include <math.h>
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

int main(void) {
  tci_stream_init();
  test_setting_and_state();
  test_trx_commands();
  test_tx_audio();
  tci_stop();
  printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
