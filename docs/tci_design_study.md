# TCI for sbitx: a plan

Status: **proposed - a plan, nothing built.** Branch `tci_for_sbitx`,
from `dbeb281`. The decisions taken so far are in §10; those still wanted
are in §11.

TCI (Expert Electronics' Transceiver Control Interface, v2.0) is one
WebSocket connection that carries text commands for control and binary
frames for receive audio, transmit audio and I/Q. JTDX, WSJT-X Improved,
MSHV, Log4OM and Hamlib's TCI backend speak it as clients; panadapters
such as sdrOxide can use its I/Q. One connection replaces CAT plus a
sound device, over the network.

maxibitx has a TCI server, built and used on the air in October 2026
(maxibitx `docs/dsp_design_notes/tci_design_study.md`, §15-17). This
plan carries it to sbitx. File and line references are to `dbeb281`.

## 1. The answer in brief

**Most of it carries over.** The WebSocket server and the stream code
know nothing about the radio, and the hard part of TCI was never the
protocol but what the clients expect without saying so (§2). Both carry
over with their tests.

**What is new is sbitx's side**, and it is about threads. Every TCI
command arrives on a network thread; sbitx's controls belong to the GTK
thread. sbitx already has the safe way across, `remote_execute()`, which
queues a command for the GTK thread to run, and the pattern for keying
the transmitter from another thread, in `hpsdr_p1.c`. TCI uses both. It
also needs to hear about changes made anywhere else in sbitx, which means
the GTK thread publishing a small snapshot of the radio's state (§5).

**A small fix comes first.** `remote_execute()`'s queue has no lock, and
three threads already write to it. TCI would be a fourth (§3.1).

**Size, as a prediction:** about 1900 lines carried over (`tci_ws.c`,
`tci_stream.c`, `tci_cw.c`, the resamplers), and about 800 new or
rewritten lines (`tci.c`'s radio side, the taps, the snapshot, settings),
plus the tests.

## 2. What carries over from maxibitx

| maxibitx file | Lines | In sbitx |
|---|---|---|
| `tci_ws.c`, `tci_ws.h` | 760 | As is: the WebSocket server (RFC 6455), a reader thread per client, a writer thread with a queue, slow clients dropped past 2 MB of binary. Decided (§10). |
| `tci_ws_test.c` | 306 | As is: 31 checks, including the 64-bit frame length that could have wrapped the size check and crashed the server. |
| `tci_stream.c`, `.h` | 552 | Nearly as is: per-client stream settings, audio rings, framing, TX_CHRONO pacing, the 24/12/8 kHz decimators. |
| `decim48k.c`, `upsample48k.c` | - | As is: 96 to 48 kHz for receive audio and I/Q, 48 to 96 kHz for transmit audio. |
| `tci.c` | 780 | Command parsing, the start-up burst, echo rules and PTT ownership carry over; every call into the radio is rewritten for sbitx (§3, §5). |
| `tci_cw.c` | 396 | The protocol half carries over; the keying half is rewritten for sbitx's CW text path (step 5, §8). |
| `tci_test.c` | 1050 | Its cases carry over; its stub radio is rewritten for sbitx's calls. |
| `tools/tci_client.py` | - | As is. |

**What the clients taught maxibitx**, all of which sbitx gets for free:

- keywords in lowercase, one command per WebSocket message;
- an echo of every set to every client, a read answered to the asker
  alone;
- `protocol` and `device` first in the start-up burst, `start;` before
  `ready;`, the whole burst in a few ms;
- a sensor value must contain a decimal point, or a client crashes;
- I/Q sent conjugated, so a station above the dial shows above it;
- transmit audio taken only from the client that holds TX with source
  `tci`, and the ring discarded on every block that isn't reading it, so a
  late answer can't reach the next transmission;
- while one client holds TX the others' `trx` is ignored, and a client
  that disconnects while transmitting releases TX.

## 3. Where TCI meets sbitx

### 3.1 Control: `remote_execute()`

`remote_execute()` (`sbitx_gtk.c:6563`) writes a command into
`q_remote_commands`; the GTK thread's tick reads it back
(`sbitx_gtk.c:10565`) and runs it with `cmd_exec()` (`sbitx_gtk.c:11952`),
on the GTK thread, where every field may safely be changed. TCI sends its
sets this way, as the same text commands the telnet console takes
(`freq 7074000`, `mode USB`, ...).

**The queue has no lock** (`queue.c:38`, `q_write()`), and
`remote_execute()` writes a command a character at a time. Threads that
already call it: the webserver's (`webserver.c:508`, `:517`), the HPSDR
poll thread (`hpsdr_p1.c:694`), and the GTK thread itself, for the
console on port 8081 (`remote.c:105`). Two of them writing at once can
interleave characters into a garbled command. A mutex around
`remote_execute()`'s loop fixes it for all of them, and is step 0.

**sbitx's own rigctld does not use it.** `hamlib.c` calls `cmd_exec()`
(`hamlib.c:201`, `:269`) and `field_set()` (`:205`, `:234`) from its
listener thread, changing GTK fields off the GTK thread. That is the
threading problem already discussed for sbitx; TCI must not copy it, and
this plan doesn't fix `hamlib.c` (it could follow later the same way).

**A set and its echo.** A command queued for the GTK thread is applied on
its next tick. TCI echoes a set at once, with the value the client sent,
as maxibitx does; the snapshot (§5) then confirms it, and if sbitx took a
different value (a frequency snapped, a mode refused), the change is sent
to everyone like any other.

### 3.2 Transmit and receive switching

`hpsdr_p1.c` already keys sbitx from a network thread: it sets a pending
action and asks the GTK thread to run it with `g_idle_add()`
(`hpsdr_p1.c:751`), which calls `tx_on(TX_SOFT)` or `tx_off()`
(`sbitx_gtk.c:8895`, `:9153`). TCI's `trx` does the same.

### 3.3 I/Q

`sound_process()` mixes the 96 kHz input to I/Q and filters it
(`sbitx.c:2248-2263`), then hands `filt_i`/`filt_q`, 1024 samples a block,
to the panadapters and to HPSDR (`sbitx.c:2280`, which passes them as
q, i). TCI's tap goes beside HPSDR's: a ring write, no lock, nothing done
if no client has started I/Q. Which way up the spectrum comes out has to
be checked on the air with a panadapter client, as it was for maxibitx
with sdrOxide; the conjugation may need to be the other way here.

### 3.4 Receive audio

The demodulated audio leaves `rx_linear()` in `output_speaker` at 96 kHz.
TCI's tap decimates it to 48 kHz (`decim48k.c`) and writes a ring; the
lower rates are made from that in the service thread, as in maxibitx.

### 3.5 Transmit audio

In DIGI, sbitx takes its transmit audio from the ALSA loopback, the
"virtual cable" WSJT-X and FLDIGI use: with `use_virtual_cable` set
(`sbitx_sound.c:1644`), `input_q` comes from the loopback queue
(`sbitx_sound.c:1222`). TCI's transmit audio goes in at the same point:
while a TCI client holds TX with source `tci`, `input_q` comes from TCI's
ring (upsampled 48 to 96 kHz) instead of the loopback. The web UI's
microphone already feeds `tx_process()` from a queue the same way
(`browser_mic_input()`, `sbitx.c:657`), which shows the idea works there.

## 4. Modes over TCI

Decided (§10): TCI offers USB, LSB, CW, CWR and DIGI:

    modulations_list:USB,LSB,CW,CWR,DIGU;

`digu` is TCI's name for upper-sideband data; it maps to sbitx's DIGI,
the wideband setting an external program uses for FT8, AM, FM and the
rest. A client setting `digu` gets DIGI; one setting `usb`, `lsb`, `cw`
or `cwr` gets that mode.

**sbitx's other modes** (FT8, FT4, AM, FM, 2TONE) are sbitx's own, chosen
on its screen. When sbitx is in one of them, TCI has to report something
in the list. Proposed: report `digu` for FT8, FT4, AM and FM, the
wideband group, and `usb` for 2TONE (§11, decision 1). A client never
sets them.

## 5. Threads and the radio's state

**Threads.** As in maxibitx: one service thread every 5 ms runs the
streams and, every tenth time, compares the radio's state with what was
last sent and broadcasts any change; a mutex serializes it with the
clients' reader threads; the audio thread never takes it, only writes
and reads lock-free rings.

**The snapshot.** TCI must tell its clients about changes made on
sbitx's screen, its knobs, the web UI, rigctld or HPSDR, and it must not
read GTK fields from its own thread to find out. So the GTK thread, once
per tick, copies what TCI reports into a small struct under a mutex:
frequency, mode, RIT on and offset, XIT on and offset, drive, volume, TX
and its source, CW speed, and the S-meter. The service thread copies it
out and compares. This is the "value snapshots" step of the threading
plan discussed earlier; once it exists, rigctld, HPSDR and the web UI
could read it too.

## 6. Coexistence

**Transmit audio,** in order of precedence when keyed:

1. the local microphone PTT, if held, keeps the microphone, as now;
2. a TCI client that holds TX with source `tci`: TCI's audio, in USB,
   LSB or DIGI;
3. otherwise as sbitx does today: in DIGI the loopback, in USB and LSB
   the microphone path.

CW with source `tci` sends no audio: CW is keyed, not played.

**PTT.** One TCI client at a time holds TX; the local key, the
microphone PTT, HPSDR and rigctld keep working as they do. A TCI client
can't unkey a transmission it didn't start.

**The loopback and TCI together.** WSJT-X on the loopback and JTDX on
TCI can both receive at once; whichever keys gets its own audio sent.

## 7. Settings

- `tci` ON or OFF, **OFF by default** (decided), and `tci_port`,
  **50001** by default (decided), Hamlib's and ExpertSDR3's default.
  Both kept in `user_settings.ini` like sbitx's other settings, and set
  from the settings screen; turning TCI on starts the server without a
  restart, turning it off disconnects every client.
- `tci_max_clients`, 4 by default (1-16), in the file only.
- The server listens on every interface; sbitx is normally on a home
  network.

## 8. Order of work, and how each step is tested

0. **Groundwork.** The mutex in `remote_execute()`; the GTK thread's
   state snapshot; the PSK31/RTTY removal (§9). Tested on the radio: the
   screen, the web UI, rigctld and HPSDR behave as before. Worth merging
   on its own.
1. **The transport.** `tci_ws.c` and `test-tci-ws`, built and run on the
   Pi. No sbitx code touched.
2. **Control.** `tci.c` for sbitx: the start-up burst, frequency, mode,
   RIT, XIT, drive, volume, the S-meter. Tested with Hamlib's
   `rigctl -m 43001`, `tools/tci_client.py`, and JTDX as CAT only; and a
   change made on sbitx's screen reaching the clients within 50 ms.
3. **Receive streams.** Receive audio and I/Q. Tested with JTDX decoding
   FT8 over TCI audio, and sdrOxide showing the spectrum the right way up.
4. **Transmit.** TCI transmit audio, `trx`, PTT ownership, the
   disconnect release. Tested into a dummy load with JTDX, then on the
   air.
5. **CW over TCI.** `cw_msg`, `cw_macros` and the rest, through sbitx's
   CW text path, read back from what is keyed.
6. **Settings screen, README and the help text.**

A first delivery would be steps 0-3, with transmit after they have been
tried (§11, decision 3).

**Tests.** sbitx has no test targets today. The plan adds a `tests/`
target in the Makefile that builds `test-tci-ws` and a `test-tci` with a
stub radio, runnable on the Pi or a PC; nothing in the radio program's
own build changes.

## 9. Removing PSK31 and RTTY

Decided (§10): they are remnants and go. Neither is offered on sbitx's
screen any more (the mode list, `sbitx_gtk.c:959`, has no PSK31 or RTTY),
but the code is still there:

| Where | What |
|---|---|
| `sdr.h:158-159` | `MODE_PSK31`, `MODE_RTTY` in `enum _mode` |
| `sbitx_gtk.c:807` | `"PSK31"`, `"RTTY"` in `mode_name[]` |
| `sbitx_gtk.c:2421-2424` | `mode_id()`'s two entries |
| `sbitx_gtk.c:8263` | macros add `^r` and key for "the fldigi modes" |
| `sbitx_gtk.c:10576` | a comment |
| `modems.c:385-386, 404, 415-421, 482-483, 539-540` | the cases that drive fldigi |
| `sbitx_sound.c:1222`, `sbitx.c:2912` | comments |
| `web/index.html:3576, 3597` | the web UI's two checks |
| `help_commands.txt:133` | the help text |

**The mode numbers must stay as they are.** The band stacks keep each
entry's mode as a number in `user_settings.ini` (`mode0=2`, ...;
`data/default_settings.ini:210`), read back as an index into
`mode_name[]`. Deleting the two enum values would renumber DIGI (10) and
2TONE (11): a saved DIGI entry would read as out of range and become USB,
and a saved RTTY entry (9) would become 2TONE, which transmits a test
tone. So the two values stay as unused slots (`MODE_UNUSED_8`,
`MODE_UNUSED_9`, with empty names never offered), and a band-stack entry
that holds one loads as USB.

**fldigi.** `modems.c` reaches fldigi over XML-RPC (`fldigi_call()`,
`modems.c:157-312`) as the modem for PSK31 and RTTY, which goes with
them. It also sends fldigi the pitch in CW, CWR, FT8 and FT4
(`modem_set_pitch()`, `modems.c:377`), which sbitx's own CW decoder and
FT8 don't need. FLDIGI over the loopback in DIGI doesn't use any of
this: it has its own rig control. Whether the pitch call goes too is
§11's decision 2.

## 10. Decisions taken

1. **Transport:** `tci_ws.c` from maxibitx, not the web UI's mongoose.
2. **Port 50001**, and **off unless enabled**.
3. **Modes:** USB, LSB, CW, CWR and DIGI (TCI's `digu`); DIGI is the
   wideband setting for FT8, AM, FM and the rest done by an external
   program.
4. **PSK31 and RTTY are removed.**

## 11. Decisions wanted

1. **What TCI reports for sbitx's own modes:** `digu` for FT8, FT4, AM
   and FM, and `usb` for 2TONE (§4)?
2. **fldigi:** remove the whole XML-RPC proxy (`modems.c:148-375`, and
   `modem_set_pitch()`'s calls), or only the PSK31/RTTY cases?
3. **First delivery:** steps 0-3 (groundwork, transport, control, receive
   streams), transmit after they have been tried?

## 12. Risks

- **The GTK tick's latency.** A set takes effect on the next tick, not
  at once. Clients tolerate that; maxibitx's TCI applied sets
  immediately, so the timing of echoes and confirmations is new here.
- **The I/Q's orientation** (§3.3) and **sbitx's audio levels** into and
  out of TCI need checking on the air.
- **CPU on the Pi 4:** the decimator, the upsampler and the I/Q ring add
  well under a percent of the audio thread's time in maxibitx; to be
  measured on sbitx, whose audio thread does more.
- **Coexistence** with the loopback, the USB microphone, HPSDR and the
  web UI's microphone, all of which can supply transmit audio (§6).
