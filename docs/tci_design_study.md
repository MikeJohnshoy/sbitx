# TCI for sbitx

Status: **steps 0-4 built** (groundwork, transport, control, receive
streams, transmit), on branch `tci_for_sbitx` from `dbeb281`. Steps 0-3
are in use on the radio with sdrOxide; transmit (step 4) passes its bench tests
(§8) and compiles into sbitx, and is next to try into a dummy load with
JTDX. CW over TCI (step 5) and the settings screen (step 6) come next.
The decisions are in §10; §11 lists what is still open.

TCI (Expert Electronics' Transceiver Control Interface, v2.0) is one
WebSocket connection that carries text commands for control and binary
frames for receive audio, transmit audio and I/Q. JTDX, WSJT-X Improved,
MSHV, Log4OM and Hamlib's TCI backend speak it as clients; panadapters
such as sdrOxide can use its I/Q. One connection replaces CAT plus a
sound device, over the network.

maxibitx has a TCI server, built and used on the air in October 2026
(maxibitx `docs/dsp_design_notes/tci_design_study.md`, §15-17). sbitx's
is carried from it.

## 1. In brief

**Most of it carried over.** The WebSocket server and the stream code
know nothing about the radio, and the hard part of TCI was never the
protocol but what the clients expect without saying so (§2). Both came
across with their tests.

**What is new is sbitx's side**, and it is about threads. Every TCI
command arrives on a network thread; sbitx's controls belong to the GTK
thread. TCI never touches a field from its own threads: it queues its
sets with `remote_execute()`, which the GTK thread runs, and it reads a
snapshot of the radio's state that the GTK thread publishes (§3.1, §5).

**`remote_execute()` now has a lock.** Its queue had none, and three
threads already wrote to it; TCI is a fourth (§3.1).

**PSK31, RTTY and the fldigi link are gone** (§9).

**Transmit** keys with sbitx's own commands through the same queue, and a
client's own audio replaces sbitx's in USB, LSB, DIGI, FT8 and FT4, even
with sbitx left in its own FT8, whose modem steps aside (§3.2, §3.5).

**Size:** about 2000 lines carried over (`tci_ws.c`, `tci_stream.c`,
`tci.c`'s protocol, `decim48k.c`), about 300 new (`tci_sbitx.c`, the
radio adapter), about 80 changed in sbitx, about 470 removed with
fldigi, PSK31 and RTTY, and 960 lines of tests.

## 2. What carried over from maxibitx

| maxibitx file | In sbitx |
|---|---|
| `tci_ws.c`, `tci_ws.h` | As is: the WebSocket server (RFC 6455), a reader thread per client, a writer thread with a queue, slow clients dropped past 2 MB of binary. |
| `tci_ws_test.c` | `tests/test_tci_ws.c`, as is: 31 checks, including the 64-bit frame length that could have wrapped the size check. |
| `tci_stream.c`, `.h` | Nearly as is: per-client stream settings, audio rings, framing, TX_CHRONO pacing, the 24/12/8 kHz decimators. It takes receive audio at +-1 (sbitx scales it, §3.4) and I/Q as given (§3.3). |
| `decim48k.c` | As is: 96 to 48 kHz for receive audio. |
| `tci.c` | Command parsing, the start-up burst and the echo rules; every call into the radio goes through `tci_radio.h` (§5), so `tci.c` holds only the protocol. |
| `tci_test.c` | `tests/test_tci.c`, rewritten for what sbitx's TCI does now, with a stub radio behind `tci_radio.h`. |
| `upsample48k.c` | As is: 48 to 96 kHz for transmit audio (§3.5). |
| `tci_cw.c` | Not yet: CW text (step 5). |

**What the clients taught maxibitx**, which sbitx gets:

- keywords in lowercase, one command per WebSocket message;
- an echo of every set to every client, with the value in force; a read
  answered to the asker alone;
- `protocol` and `device` first in the start-up burst, `start;` before
  `ready;`, the whole burst in a few ms and well under Hamlib's 256
  messages;
- a sensor value must contain a decimal point, or a client crashes;
- for transmit: audio taken only from the client that holds TX with
  source `tci`, and the ring discarded on every block that isn't reading
  it; while one client holds TX the others' `trx` is ignored, and a
  client that disconnects while transmitting releases TX.

## 3. Where TCI meets sbitx

### 3.1 Control: `remote_execute()`

`remote_execute()` (`sbitx_gtk.c`) writes a command into
`q_remote_commands`; `ui_tick()`, every millisecond on the GTK thread,
reads each command back and runs it with `cmd_exec()`, where every field
may safely be changed. Its writers are the webserver's thread, the HPSDR
poll thread, the GTK thread itself (the console on port 8081,
`remote.c`), and now TCI's reader threads.

**The lock.** The queue (`queue.c`) has no lock, and `remote_execute()`
writes a command a character at a time, so two writers at once could
interleave characters into a garbled command. `remote_execute()` now
holds a mutex while it writes a command, and `ui_tick()` holds the same
mutex while it takes one command out, then runs it without the lock, so
a command that calls `remote_execute()` itself doesn't deadlock.

**The commands TCI uses** are sbitx's own: `freq <Hz>`, `mode <name>`,
and a new one, `setfield <field id> <value>`, which sets a field through
`set_field()` exactly as the screen would. TCI needs it because the
generic `<LABEL> <value>` path moves the screen's focus to the field;
`setfield` doesn't. TCI sets drive (`tx_power`), volume (`r1:volume`),
mute (`#mute`) and CW speed (`#tx_wpm`) this way.

**A set and its echo.** A queued command is applied on the GTK thread's
next tick, a millisecond or so later, but `tci.c` echoes a set at once
with what `tci_radio_get()` returns. So `tci_sbitx.c` holds each set as
*pending*: until the published state shows the value asked for, or for
1 s if sbitx took another value (a frequency clamped, say),
`tci_radio_get()` reports the value asked for. The echo is then the new
value, and the next 50 ms comparison doesn't send the old one back
first. If sbitx did take another value, it is sent to every client like
any other change once the second is up.

**sbitx's own rigctld does not use `remote_execute()`.** `hamlib.c` calls
`cmd_exec()` and `field_set()` from its listener thread, changing GTK
fields off the GTK thread. TCI doesn't copy that; `hamlib.c` could
follow the same way later.

### 3.2 Transmit and receive switching

**Who holds TX** is `tci.c`'s: one client at a time, from its
`trx:0,true` until its `trx:0,false`, its disconnecting, the server
stopping, or TX ending some other way in sbitx (the 50 ms comparison sees
TX off while a client holds it, and releases it). While one client holds
TX, another's `trx` is answered with the state in force and does
nothing, and so is any `trx:0,true` while sbitx is already transmitting
for something else (the microphone's PTT, rigctld, sbitx's own FT8): a
TCI client can't unkey a transmission it didn't start. A repeated
`trx:0,true` from the holder changes nothing. sbitx refuses a `trx:0,true`
in 2TONE (it would send the test tones), and one with the client's own
audio in AM, FM, CW and CWR (§3.5); it is answered `trx:0,false` and
nothing is held.

**Keying** goes through `remote_execute()` like every other set, with
sbitx's own commands: `t` (`tx_on(TX_SOFT)`), `r` (`tx_off()`), and
`rtx` (`tx_on()` with the loopback's audio, as rigctld's PTT gives
WSJT-X) in DIGI, FT8 and FT4 when the client doesn't send its own audio.
`hpsdr_p1.c` uses `g_idle_add()` instead; the queue is better here because
it keeps a client's commands in order, so a frequency or mode set just
before `trx` is applied before sbitx keys. Keying takes effect on the GTK
thread's next tick; the TX state is reported at once and held, as other
sets are (§3.1), until sbitx shows it or for 1 s.

**sbitx's own FT8 and FT4** step aside while a client holds TX, and on
until sbitx has run the `r` that ends it (`tci_sbitx_tx_engaged()`):
`modem_poll()` doesn't run `ft8_poll()`, which would otherwise end the
transmission at once (it unkeys whenever it has no samples of its own,
and wipes the QSO fields as it does) or start one of its own, and
`tx_process()` doesn't take the modem's samples. So JTDX can transmit
with sbitx left in FT8, decoding in parallel.

### 3.3 I/Q

`sound_process()` (`sbitx.c`) mixes the 96 kHz input to I/Q and filters
it, then hands `filt_i`/`filt_q`, 1024 samples a block, to the
panadapters and to HPSDR. TCI's tap, `tci_sbitx_rx_block()`, sits beside
HPSDR's: a ring write, no lock, nothing done unless a client has started
I/Q.

**Which way up.** HPSDR is given them swapped, `hpsdr_send_iq(filt_q,
filt_i, ...)`, which mirrors the spectrum, as SparkSDR expects. TCI is
given them as they are, in the conventional orientation: a station above
the dial at a positive frequency. maxibitx conjugates its I/Q for TCI
because its mixer produces the opposite sense; sbitx's doesn't, so
`tci_stream.c` no longer conjugates. This is reasoning from the code,
and is to be confirmed on the air with sdrOxide (§11).

While transmitting, the tap sends silence, so a client's stream keeps
its timing.

### 3.4 Receive audio

The demodulated audio leaves `rx_linear()` in `output_speaker`, int32 at
96 kHz, before the volume control (sbitx sets the volume in the ALSA
mixer). TCI's tap scales it so int32 full scale is 1, the level the
loopback gives WSJT-X, decimates it to 48 kHz with `decim48k.c` (flat to
15 kHz, 54 dB down from 24 kHz) and writes a ring; `tci_stream.c` makes
the lower rates from that in the service thread. So TCI's audio doesn't
follow AUDIO or MUTE, as the loopback's doesn't.

### 3.5 Transmit audio

In DIGI, sbitx takes its transmit audio from the ALSA loopback: with
`use_virtual_cable` set, `input_q` comes from the loopback queue
(`sbitx_sound.c`), and `tx_process()` modulates it as it would the
microphone. A client's own audio goes in at the same point: each block,
before `tx_process()`, `sound_process()` calls `tci_sbitx_tx_block()`,
which, while a client holds TX with source `tci`, pulls the client's 48 kHz
audio from `tci_stream.c`'s ring, upsamples it to 96 kHz
(`upsample48k.c`, unity gain, images 56 dB down), scales it (below),
clips anything beyond full scale, and writes it over `input_mic`. If the client is late, the rest of the block is
silence, and once the transmission is over sbitx prints how much was
missing if it was more than 100 ms (the first and last blocks always miss
a little while keying catches up).

**The level.** A program's full scale is made the level of sbitx's own
FT8 tones, 1/21 of full scale where they reach `tx_process()`
(`ft8_next_sample()` divides by 7 and `tx_process()` by 3), which is the
level DRIVE is calibrated for. JTDX in TCI mode sends its audio at full
scale (0.999, `TCITransceiver::load()`) and does not turn its power
slider into an audio level, as it does on a sound card: it sends `drive:`
instead, 100 at 0 dB down to 0 at -45 dB (`do_txvolume()`), which sets
DRIVE (§3.1), and a DRIVE change on sbitx is sent back to it. So the
slider and DRIVE are one control, whichever moved last, and a program's
transmission comes out at the power sbitx's own FT8 would at that DRIVE.
The first build scaled a program's full scale to int32 full scale, the
loopback's, which is about 26 dB hotter: on the radio the sidetone (the
TX audio times SIDETONE, in the modes that aren't voice) was deafening,
and the transmitter was driven far past the level DRIVE is calibrated
for. The sidetone is now as loud as for sbitx's own FT8.

It takes precedence (§6, §10) in USB, LSB, DIGI, FT8 and FT4: over the
microphone, the browser's microphone, the loopback and sbitx's own FT8/FT4
modem. Being a program's line audio, it skips the voice processing
(compression, the TX EQ, CESSB, the voice clip). AM and FM take their
audio at other levels (`input_mic` / 2e8 and / 9.2e7, against / 2e9 in
the other modes), which haven't been checked against a program's, and
without the voice clip nothing would limit it; CW and CWR are keyed, not
played. A `trx` with the client's own audio is
refused in those four (§3.2) rather than sent wrongly; §11.

Between a client's `trx:0,false` and sbitx running the `r` that ends the
transmission (the GTK thread's next tick), an own-audio TX sends silence,
not whatever the microphone picks up.

While receiving, `tci_sbitx_tx_idle()` discards any audio a client sent
late, so it can't reach the next transmission, except between a client's
`trx` and sbitx actually transmitting, so the start of the client's audio
isn't lost while keying catches up. TX_CHRONO pacing keeps what is queued
to the client's `tx_stream_audio_buffering` (50 ms by default) plus one
request.

## 4. Modes over TCI

TCI offers USB, LSB, CW, CWR and DIGI:

    modulations_list:USB,LSB,CW,CWR,DIGU;

`digu` is TCI's name for upper-sideband data; it is sbitx's DIGI, the
wideband setting an external program uses for FT8, AM, FM and the rest.

sbitx's own wideband modes (FT8, FT4, AM, FM) read as `digu` too: in
them sbitx goes on receiving, demodulating and showing as usual, and the
external program works in parallel on the same audio or I/Q. So a client
setting `digu` while sbitx is in DIGI, FT8, FT4, AM or FM changes
nothing; in any other mode it selects DIGI. `usb`, `lsb`, `cw` and `cwr`
select that mode. 2TONE reads as `usb`. A mode TCI doesn't offer (`am`,
say) is answered with the mode in force.

## 5. Threads and the radio's state

**`tci_radio.h`** is everything `tci.c` needs from the radio: a state
struct (frequency, mode, TX, drive, volume, mute, RIT on and offset, CW
speed, signal strength in dBm), six sets, and keying. sbitx's side is
`tci_sbitx.c`; the tests' side is a stub.

**Threads.** As in maxibitx: one service thread every 5 ms runs the
streams and, every tenth time, compares the radio's state with what was
last sent and broadcasts any change; a mutex serializes it with the
clients' reader threads; the audio thread never takes it, only writes
lock-free rings.

**The snapshot.** TCI must tell its clients about changes made on
sbitx's screen, its knobs, the web UI, rigctld or HPSDR, and it must not
read GTK fields from its own threads to find out. So `ui_tick()` calls
`tci_sbitx_tick()` every tick, and every 10 ms, while the server runs, it
copies what TCI reports into a struct under a mutex: the FREQ, MODE,
DRIVE, AUDIO, MUTE, RIT, RIT_DELTA and WPM fields, `in_tx`, and the
S-meter's reading as dBm (S9 = -73 dBm, 6 dB an S-unit, uncalibrated).
`tci_radio_get()` copies it out. While MUTE is on, sbitx sets AUDIO to 0
and keeps the level to go back to; the snapshot reports that level as
the volume, so a client's volume control doesn't drop to the bottom.

## 6. Coexistence

**Transmit audio.** While a TCI client holds TX with its own audio, that
audio is sent instead of sbitx's own (the microphone, the browser's
microphone, the loopback, the FT8 modem) in USB, LSB, DIGI, FT8 and FT4;
in AM, FM, CW and CWR such a `trx` is refused (§3.5). Without its own
audio, sbitx sends what it would for the mode: the loopback in DIGI, FT8
and FT4, the microphone otherwise. In 2TONE every `trx` is refused. With
no client holding TX, sbitx does as it did.

**PTT.** One TCI client at a time holds TX (§3.2); the local key, the
microphone PTT, HPSDR and rigctld keep working as they do, and any of
them ending the transmission releases the client. A TCI client can't
unkey a transmission it didn't start.

**Receiving.** TCI's streams are copies: the loopback, the speaker, the
panadapters, HPSDR and sbitx's own decoders get what they got before.
WSJT-X on the loopback and JTDX on TCI can both receive at once.

## 7. Settings

- `TCI` ON or OFF, **OFF by default**, and `TCI_PORT`, **50001** by
  default (1024-65535), Hamlib's and ExpertSDR3's default. They are
  off-screen fields (`#tci`, `#tci_port`), kept in `user_settings.ini`
  like sbitx's other settings, and set with `\tci on`, `\tci off` and
  `\tci_port <port>` (help text in `help_commands.txt`) until the
  settings screen has them (step 6).
- `tci_sbitx_tick()` follows them every 100 ms: turning TCI on starts the
  server without a restart, turning it off disconnects every client, and
  a new port restarts it there. If the port can't be opened, sbitx prints
  so and tries again every 10 s.
- At most **4 clients** at once (`TCI_DEFAULT_MAX_CLIENTS`, a constant);
  a fifth is refused with HTTP 503.
- The server listens on every interface; sbitx is normally on a home
  network. `SBITX_TCI_TRACE=1` in the environment prints every message.
- sbitx stops the server as it exits (`cleanup_on_exit()`).
- While any client is connected, the frequency readout's top line, where
  the other VFO is shown, reads **TCI CONTROL ON**; with RIT or SPLIT on,
  their TX/RX line stays instead (`draw_dial()`, redrawn from `ui_tick()`
  when the count of clients goes to or from 0, `tci_client_count()`).

## 8. Order of work, and how each step is tested

0. **Groundwork - built.** The lock in `remote_execute()`, the `setfield`
   command, the snapshot, the `TCI` fields and `ui_tick()`'s call, and the
   PSK31/RTTY/fldigi removal (§9). To test on the radio: the screen, the
   web UI, rigctld and HPSDR behave as before; band stacks load.
1. **The transport - built.** `tci_ws.c`; `tests/test_tci_ws.c`, 31
   checks.
2. **Control - built.** `tci.c` for sbitx, `tci_sbitx.c`. Bench:
   `tests/test_tci.c`. To test on the radio: Hamlib's `rigctl -m 43001`,
   maxibitx's `tools/tci_client.py`, and JTDX as CAT only; a change made
   on sbitx's screen reaching the clients within 50 ms.
3. **Receive streams - built.** The taps in `sound_process()`. Bench:
   `tests/test_tci.c`. To test on the air: JTDX decoding FT8 over TCI
   audio, and sdrOxide showing the spectrum the right way up (§3.3).
4. **Transmit - built.** `trx` and who holds TX (`tci.c`), keying and
   the transmit audio block (`tci_sbitx.c`), the taps in
   `sound_process()` and `tx_process()`, FT8 stepping aside
   (`modems.c`), `upsample48k.c`. Bench: `tests/test_tci.c`,
   `tests/test_tci_sbitx.c`. To test into a dummy load: JTDX on TCI
   (Settings - Radio - TCI, audio from TCI) in DIGI and in sbitx's FT8,
   watching the power and the transmitted spectrum on a second receiver,
   then on the air; and the loopback with JTDX's audio set to the sound
   card, which keys with `rtx`.
5. **CW over TCI.** `cw_msg`, `cw_macros` and the rest, through sbitx's
   CW text path, read back from what is keyed.
6. **Settings screen and README.**

**Tests.** `make test` builds each `tests/test_*.c` with the sources it
needs and runs it; nothing in the radio program's own build changes.

- `/tmp/sbitx-test_tci_ws`: 31 checks of the WebSocket server alone.
- `/tmp/sbitx-test_tci`: 91 checks of the server with a stub radio: the
  burst's order and form; the client count; every set echoed to every client, with the value
  in force when it was out of range or not offered; reads answered to the
  asker alone; frequency, mode (uppercase accepted), drive (0 held at 1),
  volume in dB, mute and rx_mute, CW speed and speed-up; RIT, split and
  XIT answered without setting anything; several commands to a
  message; a change elsewhere reaching both clients within about 50 ms,
  and nothing sent while nothing changes; four clients and the fifth
  refused; receive audio and I/Q frames, their headers and samples, with I
  and Q as given; transmit - `trx:0,true,tci` echoed to every client and
  keying with the client's own audio, TX_CHRONO to the holder only, its
  audio reaching the audio thread in order and another client's dropped,
  another client unable to unkey or take TX, a repeated `trx`, a `trx`
  the radio refuses answered false with nothing held, release on
  `trx:0,false` with later audio dropped, `trx` with no source keying with
  sbitx's audio, `trx` ignored while sbitx transmits for something else,
  release when TX ends in sbitx, on disconnect and on stopping; stopping
  and starting again on the same port; `decim48k`'s passband and
  stopband, and `upsample48k`'s gain and images.
- `/tmp/sbitx-test_tci_sbitx`: 61 checks of `tci_sbitx.c` with sbitx's
  fields and command queue stubbed: the TCI setting starting the server;
  the state published from the fields; the command each `trx` becomes in
  each mode (`t`, `rtx`, `r`), or its refusal (own audio in AM, FM, CW,
  CWR; anything in 2TONE); the TX state reported at once, held while
  sbitx catches up, and dropped after a second if sbitx doesn't key;
  which modes take the client's audio; its level (0.5 is half the level
  of sbitx's own FT8 tones), its frequency through the upsampler,
  clipping at that level, silence when the client is late, late audio discarded while
  receiving, and, between unkeying and sbitx receiving, silence rather
  than the microphone and FT8 still standing aside.

All three pass on a PC, repeatedly. sbitx's changed files compile with no new
warnings; the whole program links against stubs for everything but
wiringPi and the FT8 library.

## 9. Removing PSK31, RTTY and the fldigi link

PSK31 and RTTY were remnants: neither was offered on sbitx's screen (the
mode list has neither), and their only implementation was fldigi, reached
over XML-RPC. sbitx no longer talks to fldigi at all.

| Where | What went |
|---|---|
| `modems.c` | The XML-RPC client (`fldigi_call()`, `fldigi_call_i()`, the base64 helpers), `fldigi_read()`, `fldigi_set_mode()`, `fldigi_tx_more_data()`, `fldigi_tx_stop()`, `modem_set_pitch()` (it only told fldigi the pitch), and every PSK31/RTTY case in `modem_rx()`, `modem_poll()` and `modem_abort()`; `modem_poll()` no longer flushes fldigi on a mode change. |
| `sdr.h` | `MODE_PSK31`, `MODE_RTTY` (see below), and `modem_set_pitch()`'s prototype |
| `sbitx_gtk.c` | the two names in `mode_name[]` and `mode_id()`; the macros' `^r` and keying "for the fldigi modes"; `modem_set_pitch()`'s calls in `do_pitch()` and in a commented-out block of `do_bandwidth()` |
| `web/index.html`, `web/style.css` | the chat TX/RX buttons shown only in PSK31 and RTTY, and the checks for those modes |
| comments, `help_commands.txt` | the mentions |

**The mode numbers stay as they are.** The band stacks keep each entry's
mode as a number in `user_settings.ini` (`mode0=2`, ...), read back as an
index into `mode_name[]`. Deleting the two enum values would renumber
DIGI (10) and 2TONE (11): a saved DIGI entry would become out of range,
and a saved RTTY entry (9) would become 2TONE, which transmits a test
tone. So the two values stay as unused slots, `MODE_UNUSED_8` and
`MODE_UNUSED_9`, with empty names that nothing offers, and a band-stack
entry that holds one, or any number no mode has, loads as USB
(`stack_mode()`).

**What stays.** fldigi as a program run over the loopback in DIGI, like
WSJT-X: it has its own rig control and never used the XML-RPC link, and
the web UI's app launcher scripts for it are untouched. `sound_input()`
stays (DIGI uses it), and so do the `STYLE_FLDIGI_RX`/`_TX` console
styles, whose numbering style files and the web UI rely on.

## 10. Decisions taken

1. **Transport:** `tci_ws.c` from maxibitx, not the web UI's mongoose.
2. **Port 50001**, and **off unless enabled**.
3. **Modes:** USB, LSB, CW, CWR and DIGI (TCI's `digu`); DIGI is the
   wideband setting for FT8, AM, FM and the rest done by an external
   program. sbitx's own FT8, FT4, AM and FM read as `digu`, and in them
   sbitx keeps receiving, demodulating and showing as usual while the
   external program works in parallel; 2TONE reads as `usb` (§4).
4. **PSK31 and RTTY are removed, and the whole fldigi link with them**
   (§9).
5. **Transmit audio precedence:** a TCI client's own transmit audio is
   sent instead of sbitx's, in whatever mode (§6, step 4). As built, that
   is USB, LSB, DIGI, FT8 and FT4; AM and FM wait on their levels (§11).
6. **First delivery:** steps 0-3; transmit after they have been tried
   (built as step 4, for FT8 first).
7. **At most 4 clients**, a constant, not a setting.
8. **RIT is reported but not set over TCI**; XIT and split report off.

## 11. Open

- **The I/Q's orientation** (§3.3): as reasoned from the code; to confirm
  with sdrOxide.
- **Audio levels** into WSJT-X and JTDX over TCI, compared with the
  loopback's; and the power a program's FT8 over TCI gives against
  sbitx's own FT8 at the same DRIVE, which should be the same (§3.5); to
  check on the air. WSJT-X Improved's TCI support comes from JTDX's, but
  that it turns its slider into `drive:` too is to confirm with
  `SBITX_TCI_TRACE=1`.
- **`hamlib.c`'s threading** (§3.1), not part of this work.
- **Seen in passing, left alone:** the web UI's `cw_keydown()` returns
  early when the mode `== "CWR"` rather than `!= "CWR"`, so typing in the
  web UI's CW window is ignored in CWR. It was so before; removing the
  PSK31 and RTTY terms kept its behaviour.
- **A program's own audio in AM and FM** (§3.5): refused for now. Sending
  it needs a level for each, matched to how JTDX or sdrOxide set their
  output, and a limit in place of the voice clip.

## 12. Risks

- **The GTK tick's latency.** A set takes effect on the next tick, a
  millisecond or so later; the pending values (§3.1) hide it from the
  clients. A tick held up by a slow redraw delays it more, still well
  within what clients tolerate.
- **CPU on the Pi 4:** the decimator, the I/Q ring and the snapshot (the
  S-meter sums 1024 bins every 10 ms) are small next to the audio
  thread's work; to be measured on the radio.
- **Coexistence** with the loopback, the USB microphone, HPSDR and the
  web UI's microphone, all of which can supply transmit audio (§6, step
  4).
