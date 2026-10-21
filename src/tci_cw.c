// tci_cw.c
//
// TCI's CW text markup to sbitx's keyer text (tci_cw.h). The characters it
// passes are those of modem_cw.c's morse_tx_table, whose < and > are the SK
// and BK prosigns: TCI's < and > are speed changes, so a literal one never
// reaches the keyer as a prosign.

#include "tci_cw.h"
#include "modem_cw.h"
#include <ctype.h>
#include <string.h>
#include <strings.h>

static const char sendable_punct[] = ".,?'!/):;-_\"@$+=&(";

static void note_skipped(char *skipped, int skip_max, char c) {
  if (!skipped || skip_max <= 1 || strchr(skipped, c))
    return;
  int n = (int)strlen(skipped);
  if (n < skip_max - 1) {
    skipped[n] = c;
    skipped[n + 1] = '\0';
  }
}

static void put(char c, char *out, int max, int *n) {
  if (*n < max - 1)
    out[(*n)++] = c;
}

static void put_plain(char c, char *out, int max, int *n, char *skipped, int skip_max) {
  if (c == '\t' || c == '\r' || c == '\n')
    c = ' ';
  if (isalnum((unsigned char)c) || c == ' ' || (c && strchr(sendable_punct, c)))
    put(c, out, max, n);
  else
    note_skipped(skipped, skip_max, c);
}

static int prosign(char a, char b) {
  static const struct {
    const char *name;
    char code;
  } table[] = {{"AR", '+'}, {"SK", '>'}, {"BT", '='}, {"KN", '('}, {"AS", '&'}, {"BK", '<'}};
  char name[3] = {(char)toupper((unsigned char)a), (char)toupper((unsigned char)b), '\0'};
  for (size_t k = 0; k < sizeof(table) / sizeof(table[0]); k++)
    if (!strcmp(name, table[k].name))
      return table[k].code;
  return -1;
}

int tci_cw_translate(const char *in, char *out, int max, char *skipped, int skip_max) {
  int n = 0;
  if (skipped && skip_max > 0)
    skipped[0] = '\0';
  if (max <= 0)
    return 0;
  for (const char *p = in; *p; p++) {
    char c = *p;
    if (c == '|') {
      const char *end = strchr(p + 1, '|');
      if (!end) {
        note_skipped(skipped, skip_max, c);
        continue;
      }
      int code = end - p == 3 ? prosign(p[1], p[2]) : -1;
      if (code >= 0)
        put((char)code, out, max, &n);
      else
        for (const char *q = p + 1; q < end; q++)
          put_plain(*q, out, max, &n, skipped, skip_max);
      p = end;
    } else if (c == '<' || c == '>') {
      put(c == '<' ? CW_SPEED_DOWN : CW_SPEED_UP, out, max, &n);
    } else {
      put_plain(c == '^' ? ':' : c == '~' ? ',' : c == '*' ? ';' : c, out, max, &n, skipped,
                skip_max);
    }
  }

  // undo the speed changes, making room for the codes if the text filled out
  int net = 0;
  for (int k = 0; k < n; k++)
    net += out[k] == CW_SPEED_UP ? 1 : out[k] == CW_SPEED_DOWN ? -1 : 0;
  while (n > 0 && n + (net < 0 ? -net : net) > max - 1) {
    char last = out[--n];
    net -= last == CW_SPEED_UP ? 1 : last == CW_SPEED_DOWN ? -1 : 0;
  }
  for (; net > 0; net--)
    out[n++] = CW_SPEED_DOWN;
  for (; net < 0; net++)
    out[n++] = CW_SPEED_UP;
  out[n] = '\0';
  return n;
}
