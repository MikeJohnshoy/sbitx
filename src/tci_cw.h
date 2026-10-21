// tci_cw.h
//
// TCI's CW text (cw_macros) in the form sbitx's keyer sends from the TEXT
// field. docs/tci_design_study.md.

#ifndef TCI_CW_H
#define TCI_CW_H

// Translates TCI text into keyer text in out (NUL-terminated, at most
// max - 1 characters):
//   - letters, digits, spaces and the punctuation sbitx's Morse table has
//     go as they are; tabs and line ends become spaces;
//   - prosigns between bars, |AR| |SK| |BT| |KN| |AS| |BK| in either case,
//     become the characters the table has for them (+ > = ( & <); letters
//     between bars that aren't one of those go as ordinary letters;
//   - < and > become CW_SPEED_DOWN and CW_SPEED_UP (modem_cw.h), and the
//     text ends with as many codes as undo them, so the next text starts at
//     WPM;
//   - ^ ~ * stand for the protocol's reserved : , ;.
// A character the table lacks is left out and added to skipped
// (NUL-terminated, at most skip_max - 1), for the caller to report.
// Returns the length written to out.
int tci_cw_translate(const char *in, char *out, int max, char *skipped, int skip_max);

#endif /* TCI_CW_H */
