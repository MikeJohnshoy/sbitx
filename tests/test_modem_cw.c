// test_modem_cw.c
//
// Bench harness for the text keyer in modem_cw.c, with sbitx around it
// stubbed: what the keyboard keyer sends from the TEXT field, measured as
// key-down lengths of the transmitted envelope - a dot at WPM, the speed
// codes (CW_SPEED_DOWN, CW_SPEED_UP) changing it by CW_SPEED_STEP for what
// follows and back, cw_stop_text() ending the text after the element in
// progress and going back to WPM, and cw_key_press_count() counting the
// key's closures. The harness plays the GTK thread (cw_poll() every
// millisecond) and the audio thread (cw_tx_get_sample() 96 times a
// millisecond) itself.
// Built and run by make test, as /tmp/sbitx-test_modem_cw.

#include "modem_cw.h"
#include "sdr.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

int cw_decode_enabled, text_ready;
static int wpm = 20, key = CW_IDLE, transmitting;
static unsigned int now_ms;
static char text[256];

int field_int(char *label) { return !strcmp(label, "WPM") ? wpm : !strcmp(label, "PITCH") ? 700 : 0; }
int get_cw_delay(void) { return 300; }
int get_cw_input_method(void) { return CW_KBD; }
int get_pitch(void) { return 700; }
int key_poll(void) { return key; }
unsigned int millis(void) { return now_ms; }
void tx_on(int trigger) {
  (void)trigger;
  transmitting = 1;
}
void tx_off(void) { transmitting = 0; }
void write_console(int style, const char *s) {
  (void)style;
  (void)s;
}
int get_tx_data_byte(char *c) {
  if (!text_ready || !text[0])
    return 0;
  *c = text[0];
  memmove(text, text + 1, strlen(text));
  int left = (int)strlen(text) + 1;
  if (!text[0])
    text_ready = 0;
  return left;
}

/* ---- The two threads ------------------------------------------------------ */

#define MAX_MARKS 64
static int marks[MAX_MARKS], n_marks; // key-down lengths, in samples

// Runs ms milliseconds, recording each key-down's length. stop_after_marks:
// call cw_stop_text() (and empty the field) once that many marks have
// started, or -1.
static void run(int ms, int stop_after_marks) {
  static int down, first, last; // in a mark; its first and last non-zero sample
  static long sample;
  for (int t = 0; t < ms; t++) {
    now_ms++;
    cw_poll((int)strlen(text), transmitting);
    for (int k = 0; k < 96; k++, sample++) {
      // a mark is a run of non-zero samples, its tone's zero crossings
      // bridged: more than 2 ms quiet ends it
      if (fabsf(cw_tx_get_sample()) > 1e-6f) {
        if (!down) {
          down = 1;
          first = (int)sample;
          if (stop_after_marks >= 0 && n_marks + 1 >= stop_after_marks) {
            text[0] = '\0';
            text_ready = 0;
            cw_stop_text();
            stop_after_marks = -1;
          }
        }
        last = (int)sample;
      } else if (down && sample - last > 192) {
        if (n_marks < MAX_MARKS)
          marks[n_marks++] = last - first + 1;
        down = 0;
      }
    }
  }
}

static void send(const char *s, int ms) {
  n_marks = 0;
  snprintf(text, sizeof(text), "%s", s);
  text_ready = 1;
  run(ms, -1);
}

// A dot at w WPM, in samples: the key-down time, and the envelope's fall
// after it (cw_envelope_data[], about 470 samples above zero), within 1%.
static int is_dot(int samples, int w) {
  double dot = 12.0 * 9600.0 / w + 470;
  return fabs(samples - dot) < 0.01 * dot;
}

void vfo_init_phase_table(void);

int main(void) {
  vfo_init_phase_table(); // sbitx.c's setup does this
  cw_init();
  run(10, -1);

  printf("the keyboard keyer's speed codes\n");
  send("E", 600);
  CHECK(n_marks == 1 && is_dot(marks[0], 20), "E at 20 WPM: one dot of %d samples", marks[0]);
  char up[] = {CW_SPEED_UP, 'E', 0}, down[] = {CW_SPEED_DOWN, 'E', 0};
  send(up, 600);
  CHECK(n_marks == 1 && is_dot(marks[0], 25), "a speed-up code first: 25 WPM (%d)", marks[0]);
  send("E", 600);
  CHECK(n_marks == 1 && is_dot(marks[0], 25), "and it stays until undone (%d)", marks[0]);
  send(down, 600);
  CHECK(n_marks == 1 && is_dot(marks[0], 20), "a speed-down code undoes it (%d)", marks[0]);
  char both[] = {'E', CW_SPEED_DOWN, CW_SPEED_DOWN, 'E', CW_SPEED_UP, CW_SPEED_UP, 'E', 0};
  send(both, 2000);
  CHECK(n_marks == 3 && is_dot(marks[0], 20) && is_dot(marks[1], 10) && is_dot(marks[2], 20),
        "E [[E ]]E: 20, 10 and 20 WPM (%d %d %d)", marks[0], marks[1], marks[2]);
  wpm = 1;
  char floor_[] = {CW_SPEED_DOWN, 'E', CW_SPEED_UP, 0};
  send(floor_, 2000);
  CHECK(n_marks == 1 && is_dot(marks[0], CW_WPM_MIN), "never below %d WPM (%d)", CW_WPM_MIN,
        marks[0]);
  wpm = 20;
  run(600, -1);

  printf("stopping the text\n");
  char slow[] = {CW_SPEED_DOWN, 'E', 'E', 'E', 'E', 'E', 0};
  n_marks = 0;
  snprintf(text, sizeof(text), "%s", slow);
  text_ready = 1;
  run(3000, 2);
  CHECK(n_marks == 2, "cw_stop_text() during the second E: the dot completes, nothing after (%d)",
        n_marks);
  send("E", 600);
  CHECK(n_marks == 1 && is_dot(marks[0], 20), "and the speed is back at WPM (%d)", marks[0]);
  char hdr[] = "H";
  n_marks = 0;
  snprintf(text, sizeof(text), "%s", hdr);
  text_ready = 1;
  run(2000, 2);
  CHECK(n_marks == 2, "stopping inside a letter: H stops after its second dot (%d)", n_marks);

  printf("the key\n");
  unsigned before = cw_key_press_count();
  run(5, -1);
  key = CW_DOT;
  run(30, -1);
  key = CW_IDLE;
  run(30, -1);
  key = CW_DASH;
  run(30, -1);
  key = CW_IDLE;
  run(600, -1);
  CHECK(cw_key_press_count() == before + 2, "two closures counted (%u)",
        cw_key_press_count() - before);

  printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
