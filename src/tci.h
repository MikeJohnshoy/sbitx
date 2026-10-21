// tci.h
//
// A TCI server (Expert Electronics' Transceiver Control Interface, v2.0):
// control, receive audio and I/Q over one WebSocket connection per client,
// for JTDX, WSJT-X Improved, MSHV, loggers, Hamlib's TCI backend and
// panadapters such as sdrOxide. Several clients at once, kept in step: a
// change made by any of them, or anywhere else in sbitx, is sent to all.
// docs/tci_design_study.md.
//
// One client at a time holds TX (tci.c), sending its own audio or
// sbitx's.

#ifndef TCI_H
#define TCI_H

#define TCI_DEFAULT_PORT 50001 // Hamlib's and ExpertSDR3's default
#define TCI_DEFAULT_MAX_CLIENTS 4

// Starts the server on bind_addr (dotted IPv4, or "" for every interface)
// and port. Returns 0, or -1 if it can't listen.
int tci_init(const char *bind_addr, int port, int max_clients);

// Disconnects every client and stops. Waits up to about a second for the
// clients' threads.
void tci_stop(void);

// Nonzero between tci_init() and tci_stop().
int tci_running(void);

// Nonzero while a client holds TX (any thread).
int tci_holds_tx(void);

// How many clients are connected (any thread).
int tci_client_count(void);

// ---- Audio thread ---------------------------------------------------------
//
// Neither blocks, locks or allocates; each returns at once when no client
// has started that stream. Transmit audio is tci_stream.h's.

// Receive audio at 48 kHz, +-1 full scale.
void tci_push_audio_rx(const double *samples, int n);

// I/Q at 96 kHz, about +-1 full scale, with a station above the I/Q's
// centre at a positive frequency. The caller centres it on
// tci_iq_center().
void tci_push_iq(const double *i_samples, const double *q_samples, int n);

// Where the clients want the I/Q centred (dds), Hz, or 0 when the server
// isn't running (any thread).
int tci_iq_center(void);

#endif /* TCI_H */
