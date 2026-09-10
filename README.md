# SensorSync-Logger

Raw event logger for post-processed GNSS timing, based on Teensy 4.1.

The board timestamps INS PPS, forwards raw NMEA, generates camera triggers, and
records exposure-active feedback. The host pairs PPS with verified NMEA time labels
and fits ticks to UTC or GPS time offline. Triggers free-run; they are not GNSS-aligned.

```text
INS PPS -> input buffer -> GPIO 19       Teensy 4.1
INS NMEA -> MAX3232 -> Serial1 RX0       GPT1 counter / FlexPWM2 outputs
Camera Trigger In <- 9 V driver <- GPIO outputs
Camera Exposure Active -> input buffers -> GPIO inputs
Teensy USB -> session_client -> raw log -> postprocess.py -> frames.csv
```

## Hardware and defaults

Refer to [the schematic](resource/SCH_Schematic1_2026-08-26.pdf). Use the Arduino
GPIO numbers below, not the schematic symbol's physical terminal numbers.

| Channel / signal | Trigger GPIO | Exposure GPIO | Default trigger rate |
|---|---:|---:|---:|
| 0: FX10E-1 | 33 | 18 | 50 Hz |
| 1: FX10E-2 | 4 | 10 | 50 Hz |
| 2: JAI-1 | 37 | 14 | off |
| 3: JAI-2 | 36 | 15 | off |
| PPS | input 19 | — | rising edge |
| NMEA | Serial1 RX: 0; TX: 1 | — | 115200, 8N1 |

The FX pair (channels 0/1) defaults to 50 Hz; the JAI pair (channels 2/3) defaults
to disabled. Each pair shares one hardware PWM rate and cannot be enabled or
disabled independently. All four wired trigger pins are parked low when their
group is disabled. Configure groups with `START freq=FX:50,JAI:0`. Channel counts are derived
from the configuration arrays; do not add empty entries. GPIO conflicts involving
trigger outputs, Serial1, PPS and exposure inputs are rejected at compile time.

Exposure polarity currently remains `active_high=true` on all channels. **Verify
this against each camera's model and output configuration before using exposure
times.** The schematic has camera-side 1 kΩ pull-ups and non-inverting buffers:
a disconnected camera reads HIGH, regardless of the Teensy's internal pull-down.
A camera that sinks the line during exposure needs `active_high=false` in
`STROBE_CFG`. Channel 2 belongs to JAI1 on GPIO 14; channel 3 belongs to JAI2 on GPIO 15.

Teensy GPIO accepts 0–3.3 V and is not 5 V tolerant. RS-232 must pass through the
MAX3232. U501–U503 accept the 5 V signals and output 3.3 V logic; their inputs are
specified up to 5.5 V. ([PJRC](https://www.pjrc.com/store/teensy41.html),
[TI](https://www.ti.com/product/SN74LVC2G17))

U401 supplies approximately 9 V trigger pulses. Confirm the actual camera's input
voltage/current requirements; its low-going edge depends on the 4.7 kΩ pull-down,
cable capacitance and camera load. Measure both edges and the pulse width at the
camera connector. The MCU's 5 V rail comes from USB through the Teensy's normal
VUSB–VIN connection; the schematic's 12 V input supplies the trigger driver through
U402. Do not treat a 12 V connection alone as MCU power.

## Timing model

GPT1 runs at nominally 75 MHz, software-extended to 64 bits across its approximately
57.3 s rollover. FlexPWM2 generates the two output pairs independently of ISR
latency. The requested pulse width defaults to 50 µs and is quantized to the PWM
clock; `#GROUP` records the actual timing. PWM interrupts sample GPT1 for `T`
records. Detected missing/ambiguous PWM observations or trigger-queue overflow
park the outputs and increment `pwmerr`; these fail the host recording.

Current log timestamps mean:

| Stream | Timestamp meaning |
|---|---|
| `P` | GPIO PPS ISR sample |
| `T` | GPT sample in the PWM interrupt handler; both members of a pair share it |
| `S` | GPIO exposure ISR sample |
| `Z` | Time when the NMEA terminator is read by the main loop |

The 13.3 ns tick is **resolution, not guaranteed accuracy**. ISR latency and external circuitry contribute
measurement error. PPS fitting corrects the clock scale/drift; it cannot remove
these path delays. The current firmware uses GPIO PPS only; no hardware-capture
switch or self-test is provided. Older logs used scheduled compare values for most
`T` records; check their `#TIME` declaration before comparing latency measurements.

## Build and flash

With PlatformIO installed:

```bash
./upload.bash -b        # build only
./upload.bash           # build, flash, then check the idle banner
```

`upload.bash` refuses to flash while a reader holds the port. It selects the CLI
loader when no DISPLAY is set. `pio run` builds the default environment;
`pio run -e teensy41_cli` builds the headless upload environment.

The GPT timebase changes the shared peripheral clock. Do not add IntervalTimer or
other PIT users that assume the original 24 MHz clock without revisiting this setup.

On Linux, install [PJRC's udev rules](https://www.pjrc.com/teensy/00-teensy.rules)
and grant the recording user serial-port access (normally the `dialout` group).
An acquisition host needs a C++20 compiler to build the recorder, but does not need
PlatformIO unless it also builds/flashes firmware.

## Recording

The host client opens the port in raw/no-echo mode and synchronizes it **before
the camera is armed**. It terminates a residual command line, sends STOP, then
uses unique unknown-command echoes before and after an idle status request.
The final echo confirms the status command was consumed, so a periodic idle
heartbeat cannot leave its reply pending for the capture reader. Existing
firmware handles these commands; no new firmware command is
required. Missing barrier/idle, a disconnect, an oversized reply or a non-idle
reply after the barrier fails preparation without sending a new START. Serial
I/O during preparation has a three-second deadline (poll granularity 100 ms).

Cleanup replies, including a possible `#ERR,unknown_command,# SensorSync-logg`
and the intentional `#ERR,unknown_command,__SENSORSYNC_SYNC_...` barrier, are
startup diagnostics, not recording errors. The client retains at most 64 KiB in
`startupLog()`. FX10 writes this transcript to its application log; the standalone
client writes it to stderr (`<log>.stderr` when launched by `record_ctl.bash`).
`sensor_trigger.log` still contains only the subsequent session stream, whose
`#ERR`, unexpected IDLE, loss and restart checks remain strict.

Opening the port now sends STOP and requires idle. Only the owner of the board
may do this, before acquisition; do not open a second client or serial monitor
against a running capture. This preparation does not implement automatic recovery
during a session or validate the integrity of a previous session it stops.

Build the shared C++ client once, and rebuild it after source changes:

```bash
g++ -std=c++20 -O2 -pthread -DSESSION_CLIENT_DEMO session_client.cpp -o session_client
./session_client /dev/ttyACM0 /tmp/run.log 5 25
# Arguments: port, new log path, seconds (0 = until signal), FX Hz or freq=FX:hz,JAI:hz
```

The client creates a new file exclusively, waits for protocol v2 `#READY`, and
on SIGINT/SIGTERM sends STOP, waits for final health and STOP acknowledgement, then
flushes and synchronizes the file. A USB disconnect, reboot, unexpected IDLE,
repeated START, protocol error or reported loss fails the run. It does not reconnect
or restart the board automatically; end the associated camera acquisition epoch.
The standalone client exits nonzero on failure.

`record_ctl.bash` uses this same executable and protocol handling:

```bash
export SENSOR_PORT=/dev/serial/by-id/usb-Teensyduino_... # replace with the actual port
LOG=$(./record_ctl.bash start /data/logs FX:25.5,JAI:0) || exit 1
./record_ctl.bash status
# ... acquire camera data ...
./record_ctl.bash stop || exit 1
```

`start` prints a unique absolute log path only after `#READY` confirms PWM startup.
`#SESSION,START` alone is insufficient; firmware without v2 `#READY` fails startup.
`stop` reports success only after the client has validated STOP and finalized the
file; an unconfirmed or lossy run returns nonzero. `new` stops the previous run and
starts another only if the previous run completed successfully. No automatic START
is issued during recovery or stopping. A failed run's state remains available until
`stop` collects the result. Sidecars `<log>.status` and `<log>.stderr` retain the
recorder result and diagnostics.

`SENSOR_PORT` selects the port explicitly. If unset, exactly one Teensy by-id port
(or one ttyACM fallback) must exist. `SESSION_CLIENT` overrides the executable path;
`RECORD_STATE` overrides `/tmp/sensorsync_record.state`. Keep one reader per port;
do not run a monitor or a second recorder alongside it.

For C++ integration, include `session_client.cpp` without `SESSION_CLIENT_DEMO`:

```cpp
SensorSyncSession s;
if (!s.open(port) || !s.start(log_path, {{0, 25.0}, {2, 5.0}})) {
    // Abort camera acquisition: timing session did not start successfully.
}
// During acquisition, poll s.ok(); end the camera epoch if it becomes false.
s.stop();
if (!s.ok()) {
    // Preserve the log for diagnosis; the run did not complete cleanly.
}
```

The C++ API retains physical channel IDs: 0/1 configure the FX pair, and 2/3
configure the JAI pair. The client converts these to named groups on the wire
and rejects conflicting rates within a pair before creating a recording file.

## Board protocol and configuration

USB CDC commands are newline-terminated; 115200 is a nominal USB baud setting.
The board boots idle. `config.h` contains pins, defaults, pulse widths and
interrupt priorities.

| Command | Action |
|---|---|
| `START [freq=FX:HZ,JAI:HZ] [path]` | Validate rates, close any old session, emit header, start PWM and acknowledge with `#READY` |
| `STOP` | Disable outputs/capture, drain queued events, emit final health and STOP |
| `s` or `STATUS` | Health while running; IDLE otherwise |
| `h` or `HEADER` | Reprint the current channel and timebase configuration |

A frequency of 0 disables a whole pair. Enabled FX rates must be finite and at
least 20 Hz; JAI rates must be within 1–10 Hz. The firmware also checks whether
the hardware clock, period and pulse width can represent the request. Omitted
groups retain their previous rate until reboot. Numeric wire keys such as
`freq=0:50` are rejected. Invalid requests emit `#ERR,bad_freq` without changing
the existing session or rates. Check `#GROUP` and `#TRIG` for the quantized rate.
The path is echo-only; it does not make the board write to the host filesystem.

| Record | Meaning |
|---|---|
| `#SESSION,START,<path>` / `#SESSION,STOP,<path>` | Session boundaries |
| `#LOG,SensorSync-logger,2,tick_hz=<f>` | Protocol version and nominal timebase frequency |
| `#GROUP,<name>,requested=...,actual=...,pulse_us=...,clock_hz=...,div=...,period=...,width=...` | Requested and actual hardware PWM timing |
| `#TRIG,<i>,<name>,<pin>,<freq>,ah=<0|1>` | Trigger configuration |
| `#STROBE,<i>,<name>,<pin>,ah=<0|1>` | Exposure configuration |
| `#TIME,...` | Timestamp sampling semantics |
| `#READY` | Header completed and hardware PWM started |
| `P,<seq>,<tick>` | PPS event |
| `T,<seq>,<ch>,<level>,<tick>` | PWM interrupt observation of a trigger edge |
| `S,<seq>,<ch>,<level>,<tick>` | Exposure edge |
| `Z,<seq>,<tick>,<raw NMEA>` | Complete NMEA sentence |
| `#H`, `#Ht`, `#Hs` | Periodic aggregate and per-channel health |
| `#HFINAL ...` | Final aggregate counters before STOP |
| `#TRUNC,<group>,<last-rise-seq>,<stop-tick>` | STOP cut a high pulse; no synthetic falling edge is emitted |
| `#IDLE,up=<ms>` | Idle heartbeat |

Sequence numbers start at 1 per stream/channel each session and advance on software
queue/output drops. They do not count physical edges missed before the GPIO ISR or
bytes lost before sentence assembly. Data output drops rather than blocking when
USB has no space; the event rings do not guarantee a host-stall recording buffer.

Health fields include `pps`, `ppsdt` (last measured interval, not PPS age), `tod`,
`todb`, `ppsdrops`, `todtrunc`, `tdrops`, `sdrops`, `host`, `pwmerr`, and `trunc`.
Final health preserves counters before reset. The recorder rejects nonzero loss
or PWM-error counters. A STOP cut is accepted only with valid `#TRUNC` records
matching the final `trunc` count. STOP ends capture immediately, so an exposure
still in progress can lack its closing edge; post-processing flags that observation.

## Post-processing

Requires Python and NumPy. Both the receiver's NMEA time scale and its PPS-label
convention must be verified; neither is inferred from sentence proximity.

```bash
# Example only: use these values after verifying UTC labels following their PPS.
python3 postprocess.py "$LOG" -o "${LOG%.log}.csv" --tod-scale utc --convention preceding
# Add --gps --leap N for GPS output, replacing N with the verified GPS-UTC offset.
```

| Option | Meaning |
|---|---|
| `--tod-scale utc|gps` | Required: receiver calendar-field time scale |
| `--convention preceding|following` | Required: which PPS the received label describes |
| `--gps` | Output GPS seconds instead of UTC/Unix seconds |
| `--leap N` | Explicit GPS−UTC offset; required for GPS input or output |
| `--window S` | Fit window, default 10 s |
| `--session N` | Required selection if a log contains multiple sessions |
| `--link A:B,...` | Strobe-to-trigger map; default same channel index |
| `--edges FILE` | Optional per-edge CSV |
| `-o FILE` | New per-frame CSV, default `frames.csv`; existing files are not overwritten |

The fit uses small numeric offsets to preserve timestamp precision. A centered
result requires coverage of the full requested window, retained anchors before
and after the event, and no missing-second gap among retained anchors. Session
edges, sparse regions and rejected anchors may produce a valid but non-centered
fit. Missing time coverage leaves affected output times blank.

Trigger matching considers only triggers at or before exposure start, within half
a trigger period. Out-of-window, duplicate-time or reused candidates stay unmatched.
This remains a heuristic, not camera BlockID verification: `association_verified=0`.
An exposure delayed beyond that window stays unmatched.

CSV includes start/end/midpoint, `dur_us`, matched trigger time, latency, fit RMS,
anchor count, source sequences/ticks, flags, and the selected time-scale settings.
`dur_us` and latency use the same calibrated times as the timestamp columns.
Duration is blank unless both exposure endpoints have valid fits.

| Flag | Meaning |
|---|---|
| `0x01` | Incomplete time fit |
| `0x02` | No matched trigger time, or its fit failed |
| `0x04` | Suspect exposure reconstruction near a lost edge |
| `0x08` | Non-centered fit / incomplete window coverage |
| `0x10` | Missing exposure end edge |
| `0x40` | Log integrity incomplete or unverified |

Fit RMS describes residuals; it is not an absolute timing-accuracy guarantee.

## Validation

```bash
python3 -m unittest discover -s tests -v
./upload.bash -b
```

Host regression tests exercise four-channel scheduling, wraparound, delayed ISRs,
frame matching, fit coverage, calibrated duration, and recorder acknowledgements
using a simulated serial port. They do not replace board measurements.

On the bench, check each camera separately before combining channels. Verify PPS
and NMEA arrival, polarity, trigger voltage/pulse width, and the camera's accepted
frame count. Run longer than 120 s to cross multiple GPT rollovers. Check the
integer-second time against a trusted reference and measure the trigger/feedback
path with the actual cables and camera load.

For **bare Teensy only**, disconnect external drivers before jumpering GPIO 33 to
GPIO 18 for a logic-level loopback. On the assembled board, the exposure buffer
already drives GPIO 18; do not bridge two outputs. Never connect a 9 V camera
trigger output directly to a Teensy pin. GPIO 41 is the schematic's 12 V sense
input, not an exposure input.

## Files

`SensorSync.ino` controls sessions and logging; `timebase.*` provides the GPT clock;
`channels.*` schedules triggers and captures strobes; `timesync.*` queues PPS and
assembles NMEA. `postprocess.py` converts logs. `session_client.cpp` and
`session_protocol.h` implement host recording; `record_ctl.bash` manages its
lifecycle. `upload.bash` builds/flashes; `tests/` contains host regression tests.

Released under the [MIT License](LICENSE).
