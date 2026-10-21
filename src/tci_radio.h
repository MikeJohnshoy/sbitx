// tci_radio.h
//
// What the TCI server (tci.c) needs from the radio, and nothing else, so
// tci.c holds only the protocol. sbitx's side is tci_sbitx.c; the tests'
// side is a stub (tests/test_tci.c).
//
// Every call is made on a TCI thread, never the audio thread or the GTK
// thread. A set may take effect a moment later, but tci_radio_get() reports
// it at once: tci.c echoes a set with what tci_radio_get() then returns.

#ifndef TCI_RADIO_H
#define TCI_RADIO_H

// The modes TCI offers (modulations_list): USB, LSB, CW, CWR, and DIGU for
// sbitx's DIGI. sbitx's own wideband modes (FT8, FT4, AM, FM) read as
// DIGU, and 2TONE as USB.
enum tci_mode { TCI_MODE_USB, TCI_MODE_LSB, TCI_MODE_CW, TCI_MODE_CWR, TCI_MODE_DIGU };

#define TCI_DRIVE_MIN 1
#define TCI_DRIVE_MAX 100
#define TCI_WPM_MIN 1
#define TCI_WPM_MAX 50

struct tci_radio_state {
  int freq;          // the dial, Hz
  enum tci_mode mode;
  int tx;            // transmitting, from any source
  int drive;         // TCI_DRIVE_MIN to TCI_DRIVE_MAX
  int volume;        // 0-100
  int mute;
  int rit_on, rit;   // RIT and its offset, Hz
  int wpm;           // CW speed
  double dbm;        // signal strength, dBm, S9 = -73 (uncalibrated)
};

void tci_radio_get(struct tci_radio_state *s);
void tci_radio_set_freq(int hz);
void tci_radio_set_mode(enum tci_mode m);
void tci_radio_set_drive(int drive);
void tci_radio_set_volume(int percent);
void tci_radio_set_mute(int on);
void tci_radio_set_wpm(int wpm);

// Key (on) or unkey the transmitter for the client that holds TX. own_audio:
// the client sends its audio (tci_stream.c); otherwise sbitx's usual audio
// for the mode is used. Returns 0, or -1 if the radio refuses to key in its
// present mode (unkeying is never refused).
int tci_radio_set_tx(int on, int own_audio);

#endif /* TCI_RADIO_H */
