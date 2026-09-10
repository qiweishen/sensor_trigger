# SensorSync-Logger

Raw event logger for post-processed GNSS timing, based on Teensy 4.1.

Two FX10 cameras share one hardware PWM frequency; two JAI cameras share another.
The two group rates are independently configurable at session start:

| Group | Enabled frequency range | Default after START | Control granularity |
|---|---|---|---|
| `FX` | ≥20 Hz, subject to pulse/counter and camera limits | 50 Hz | Both FX10 cameras together |
| `JAI` | 1–10 Hz | off | Both JAI cameras together |

Zero disables a group. Within each group, both outputs use the same phase and
pulse width. The board boots with all trigger outputs LOW and waits for START.
See [Recording](#recording) for host commands and
[the implementation plan](HARDWARE_PWM_PLAN.md) for design details.

The board timestamps INS PPS, forwards raw NMEA, generates camera triggers, and
records exposure-active feedback. The host pairs PPS with verified NMEA time labels
and fits ticks to UTC or GPS time offline. Triggers free-run; they are not GNSS-aligned.

```text
INS PPS -> input buffer -> GPIO 19       Teensy 4.1
INS NMEA -> MAX3232 -> Serial1 RX0       GPT1 timestamp counter
Camera Trigger In <- 9 V driver <- FlexPWM2 SM0 / SM3 outputs
Camera Exposure Active -> input buffers -> GPIO inputs
Teensy USB -> session_client -> raw log -> postprocess.py -> frames.csv
```

## Hardware and defaults

Refer to [the schematic](resource/SCH_Schematic1_2026-08-26.pdf). Use the Arduino
GPIO numbers below, not the schematic symbol's physical terminal numbers.

| Log channel | Group | Camera | Trigger GPIO | FlexPWM2 output | Exposure GPIO |
|---:|---|---|---:|---|---:|
| 0 | `FX` | FX10E-1 | 33 | SM0 B | 18 |
| 1 | `FX` | FX10E-2 | 4 | SM0 A | 10 |
| 2 | `JAI` | JAI-1 | 37 | SM3 B | 14 |
| 3 | `JAI` | JAI-2 | 36 | SM3 A | 15 |

PPS uses GPIO 19, rising edge. NMEA uses Serial1 RX on GPIO 0 (TX on GPIO 1),
115200 baud, 8N1. The four numeric channel IDs remain in logs and exposure matching;
frequency commands use the group names `FX` and `JAI`.

Build-time settings are in [config.h](config.h):

| Setting | Current value | Purpose |
|---|---|---|
| `TRIGGER_DEFAULT_RATES` | `{50.0, 0.0}` in FX, JAI order | Requested rates before the first override |
| `GROUP_PULSE_US` | `{50, 50}` in FX, JAI order | Requested minimum HIGH pulse widths, in µs |
| `STROBE_CFG[].active_high` | `true` on all four channels | Exposure-active polarity used in post-processing |
| `LED_HOLD_MS` | `10` | Activity LED hold time, in ms |

Only group frequency is adjustable through START. Changing pulse width or exposure
polarity requires rebuilding and flashing the firmware. Per-camera frequency,
phase and enable overrides are unsupported. The driver checks the fixed Teensy
4.1 PWM pin mapping at compile time.

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

## Activity LEDs

All six signal LEDs are active HIGH through 330 ohm resistors (R211-R216).
Their pin numbers below are Arduino GPIO numbers, not U201 terminal numbers.

| LED | GPIO | Activity during START..STOP |
|---|---:|---|
| D211 / LED_PPS | 2 | Observed PPS edges |
| D212 / LED_RS232 | 3 | Serial1 RX bytes read, including incomplete or invalid NMEA |
| D213 / LED_TRIG_FX | 6 | Observed Trigger events on the FX pair (channels 0, 1) |
| D214 / LED_EXP_FX | 5 | Exposure input edges on either FX channel (0, 1) |
| D215 / LED_TRIG_JAI | 8 | Observed Trigger events on the JAI pair (channels 2, 3) |
| D216 / LED_EXP_JAI | 7 | Exposure input edges on either JAI channel (2, 3) |

Each observed counter change lights its LED for `LED_HOLD_MS` (default 10 ms)
after the main loop notices it; further activity extends the hold. Fast continuous
signals therefore appear steadily lit. These are activity indicators, not exact
pulse mirrors, GNSS lock indicators, or proof of valid NMEA or successful logging.
Both trigger/exposure edge polarities count. A disconnected exposure input held
HIGH by the board's pull-up produces no continuing activity indication.

The LEDs start off before the USB connection wait, remain off while idle, and turn
off immediately on STOP before draining the log tail. START clears their history.
`leds::poll()` reads existing counters and changes GPIO only in the main loop;
it adds no timer, delay, interrupt work or interrupt masking. Queue/USB drops do
not hide counted activity. Refresh timing follows the main loop, so command/header
waits can extend a visible flash. Pin conflicts with capture and UART are rejected
at compile time.

D111 (+12V, R111 2.2 kohm), D112 (+5V, R112 680 ohm) and D113 (+3V3, R113
330 ohm) are connected directly to their power rails and need no firmware.
The Teensy's built-in GPIO 13 LED is not one of these schematic signal LEDs.

## Timing model

GPT1 runs at nominally 75 MHz, extended to 64 bits across its approximately
57.3 s rollover. It records time and no longer schedules trigger edges.

At the configured 600 MHz CPU / 150 MHz IPG bus clock:

```text
IPG 150 MHz
  +-- PERCLK /2 ---------------------- GPT1 75 MHz: event timestamps
  +-- FlexPWM2 SM0 /128: 1.171875 MHz
        +-- SM0 counter + A/B -------- FX2 / FX1
        +-- AUX_CLK --> SM3 /D
                         +-- A/B ----- JAI2 / JAI1
```

FlexPWM2 SM0 drives FX2/FX1 on A/B. Its fixed IPG /128 clock is 1.171875 MHz.
SM3 drives JAI2/JAI1 on A/B, using SM0 AUX_CLK followed by its own prescaler.
The prescaler is selected to fit JAI's 1–10 Hz period in 16 bits. Both submodules
use local initialization/reload and identical A/B compare values. Changing FX's
period cannot retune JAI: AUX_CLK is the SM0 count clock, not its trigger frequency.
**SM0 keeps running even with FX off when JAI is on.** Disabled trigger outputs
remain LOW GPIO, including the FX pair in JAI-only mode. CPU/IPG clock changes and
`analogWriteFrequency()` on these pins are unsupported.

This internal clock route is documented in [NXP's reference manual, section
55.3.2.1](https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf#page=3107).
See [the implementation plan](HARDWARE_PWM_PLAN.md) for calculations and bench checks.

Period and width are rounded up to whole PWM ticks. Requested frequencies are
FX ≥20 Hz (subject to pulse/counter limits) and JAI 1–10 Hz; 0 means off. The
firmware's representable FX limit is not a camera frame-rate rating or a verified
logging throughput limit. The caller must select a rate supported by the camera.
A pulse starts one quantized pulse-width interval after counter start. Rate changes
take effect in a new session and restart the counters; continuous phase across
sessions and cross-group phase locking are not provided.

| Requested setting | Nominal actual frequency | Actual pulse for a 50 µs request |
|---|---:|---:|
| FX 20 Hz | 19.99991467 Hz | 50.346667 µs |
| FX 25 Hz | 25.00000000 Hz | 50.346667 µs |
| FX 50 Hz | 49.99893336 Hz | 50.346667 µs |
| JAI 1 Hz | 0.99997525 Hz | 54.613333 µs |
| JAI 2 Hz | 1.99995051 Hz | 54.613333 µs |
| JAI 5 Hz | 4.99997867 Hz | 51.200000 µs |
| JAI 10 Hz | 9.99995733 Hz | 51.200000 µs |

These are calculated values at a nominal 150 MHz bus clock, not bench measurements.
Confirm the actual pulse width is acceptable to each camera. Hardware comparisons
generate both edges even when the CPU is servicing other interrupts.

Current log timestamps mean:

| Stream | Timestamp meaning |
|---|---|
| `P` | GPIO PPS ISR sample of GPT1 |
| `T` | PWM event handler sample of GPT1, shared by the two cameras in a group |
| `S` | GPIO exposure ISR sample of GPT1 |
| `Z` | GPT1 sample when the NMEA terminator is read in the main loop |

PWM IRQs both use priority 48, below GPT1 at 32 and above GPIO at 64. They cannot
preempt each other while writing the shared Trigger queue. STOP freezes counters,
handles final pending PWM flags under an interrupt guard, then takes the output
pins back as LOW GPIO. A HIGH pulse interrupted by STOP is reported by `#TRUNC`,
without inventing a T falling-edge timestamp. Its last rising sequence is retained.

Merged compare flags, invalid edge order, a missing clock/IRQ, or a full Trigger
queue fail the acquisition: outputs go LOW, `pwmerr` increments and the session
closes with `#ERR,pwm_event_loss`. A new explicit START resets the fault. Counts
reflect observed events, not an independently captured physical pulse count.

The 13.3 ns GPT tick is **resolution, not guaranteed accuracy**. T is an interrupt
sample, including pending events handled during STOP, and is not a hardware latch
or an inferred compare deadline. Interrupt latency and external paths remain in
P/T/S observations. PPS fitting cannot remove them. Hardware PWM improves pulse
stability; it does not by itself improve absolute timestamp accuracy. Older logs
used GPIO write samples or scheduled compare times; inspect `#TIME`.

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

The current firmware uses **protocol version 2**. Rebuild the host client and flash
the matching firmware when upgrading from version 1. The current client rejects
older firmware; historical version 1 log files remain readable by `postprocess.py`.

Build the shared C++ client once, and rebuild it after source changes:

```bash
g++ -std=c++20 -O2 -pthread -DSESSION_CLIENT_DEMO session_client.cpp -o session_client
./session_client /dev/ttyACM0 /tmp/run.log 5 freq=FX:25,JAI:5
# Arguments: port, new log path, seconds (0 = until signal), optional named group rates
```

The client creates a new file exclusively and waits for `#READY` after the complete
header and hardware startup. `#SESSION,START` alone does not confirm readiness.
On SIGINT/SIGTERM, the client sends STOP, waits for final health and STOP
acknowledgement, then flushes and synchronizes the file. A USB disconnect, reboot,
unexpected IDLE, repeated START, protocol error or reported loss fails the run.
It does not reconnect or restart the board automatically; end the associated camera
acquisition epoch. The standalone client exits nonzero on failure.

`record_ctl.bash` uses this same executable and protocol handling:

```bash
export SENSOR_PORT=/dev/serial/by-id/usb-Teensyduino_... # replace with the actual port
LOG=$(./record_ctl.bash start /data/logs FX:25.5,JAI:5) || exit 1
./record_ctl.bash status
# ... acquire camera data ...
./record_ctl.bash stop || exit 1
```

The frequency argument differs between entry points:

| Entry point | Example enabling both groups |
|---|---|
| Board USB command | `START freq=FX:25,JAI:5` |
| Standalone recorder | `./session_client /dev/ttyACM0 /tmp/run.log 5 freq=FX:25,JAI:5` |
| Recording wrapper | `./record_ctl.bash start /data/logs FX:25,JAI:5` |

The wrapper adds `freq=` itself; do not include that prefix in its argument.
Use an explicit value for both groups when starting a reproducible acquisition:

| Desired operation | Wrapper frequency argument | Equivalent board command |
|---|---|---|
| FX pair at 50 Hz, JAI pair at 5 Hz | `FX:50,JAI:5` | `START freq=FX:50,JAI:5` |
| FX pair only, 50 Hz | `FX:50,JAI:0` | `START freq=FX:50,JAI:0` |
| JAI pair only, 1 Hz | `FX:0,JAI:1` | `START freq=FX:0,JAI:1` |
| Log PPS/NMEA/exposure inputs with triggers off | `FX:0,JAI:0` | `START freq=FX:0,JAI:0` |

Stop the current recording before changing rates. The wrapper's `new` command
performs a stop followed by a new recording; it does not retune an active session.

`start` prints a unique absolute log path only after the hardware `#READY` acknowledgement.
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
if (!s.open(port) || !s.start(log_path, {{trigger::Group::FX, 25.0}, {trigger::Group::JAI, 5.0}})) {
    // Abort camera acquisition: timing session did not start successfully.
}
// During acquisition, poll s.ok(); end the camera epoch if it becomes false.
s.stop();
if (!s.ok()) {
    // Preserve the log for diagnosis; the run did not complete cleanly.
}
```

## Board protocol and configuration

USB CDC commands are newline-terminated; 115200 is a nominal USB baud setting.
The board boots idle. `config.h` contains pins, group defaults, pulse widths and
interrupt priorities.

| Command | Action |
|---|---|
| `START [freq=FX:HZ,JAI:HZ] [path]` | Validate the whole request, close any old session, reset, emit header, start hardware, send READY |
| `STOP` | Disable outputs/capture, drain queued events, emit final health and STOP |
| `s` or `STATUS` | Health while running; IDLE otherwise |
| `h` or `HEADER` | Reprint the current channel and timebase configuration |

A frequency of 0 disables a group. Omitted groups retain their prior requested
rate until reboot; omitting the entire `freq=` option retains both groups. An
explicitly empty `freq=` is invalid. Duplicate groups, old numeric channel IDs,
nonfinite/out-of-range values and unrepresentable pulse/period settings reject the
entire START with `#ERR,bad_freq`, leaving an existing session and rates intact.
Check `#GROUP` and the four `#TRIG` entries for actual quantized settings. The path
is echo-only; it does not make the board write to the host filesystem.

Old commands such as `freq=0:25,1:25` and the old bare numeric CLI argument are
rejected. Group names are uppercase and case-sensitive.

| Record | Meaning |
|---|---|
| `#SESSION,START,<path>` / `#SESSION,STOP,<path>` | Session boundaries |
| `#LOG,SensorSync-logger,2,tick_hz=<f>` | Nominal timebase frequency |
| `#READY` | Hardware enabled after a complete session header |
| `#GROUP,<name>,...` | Group name (`FX` or `JAI`), requested/actual rate, actual width, clock, divider and compare counts |
| `#TRIG,<i>,<name>,<pin>,<freq>,ah=<polarity>` | Trigger configuration; polarity is 0 or 1 |
| `#STROBE,<i>,<name>,<pin>,ah=<polarity>` | Exposure configuration; polarity is 0 or 1 |
| `#TIME,...` | Timestamp sampling semantics |
| `P,<seq>,<tick>` | PPS event |
| `T,<seq>,<ch>,<level>,<tick>` | Trigger edge |
| `S,<seq>,<ch>,<level>,<tick>` | Exposure edge |
| `Z,<seq>,<tick>,<raw NMEA>` | Complete NMEA sentence |
| `#H`, `#Ht`, `#Hs` | Periodic aggregate and per-channel health |
| `#TRUNC,<group>,<rise_seq>,<stop_tick>` | STOP shortened the pair's final HIGH pulse; group 0=FX, 1=JAI, tick is sampled after parking outputs |
| `#HFINAL ...` | Final aggregate counters before STOP |
| `#IDLE,up=<ms>` | Idle heartbeat |

In `#GROUP`, `clock_hz` is that group's PWM counter frequency. FX `div=128` is the
SM0 divider from IPG; JAI `div` is the additional SM3 divider from AUX_CLK.
`period` and `width` are PWM counts, not GPT1 timestamp ticks. Read `actual` and
`pulse_us` for the resulting nominal frequency and pulse width.

Sequence numbers start at 1 per stream/channel each session and advance on software
queue/output drops. They do not count physical edges missed before the GPIO ISR or
bytes lost before sentence assembly. Data output drops rather than blocking when
USB has no space; the event rings do not guarantee a host-stall recording buffer.

Health fields include `pps`, `ppsdt` (last measured interval, not PPS age), `tod`,
`todb`, `ppsdrops`, `todtrunc`, `tdrops`, `sdrops`, `host`, `pwmerr`, and `trunc`.
Final health preserves loss counters before reset. The recorder rejects nonzero
loss/PWM-error counters. `trunc` is informational when every cut has a matching
`#TRUNC` record; the affected frame is flagged separately. STOP ends capture
immediately, so an exposure still in progress can lack its closing edge;
post-processing flags that observation.

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
| `--tod-scale SCALE` | Required: receiver calendar-field time scale, `utc` or `gps` |
| `--convention MODE` | Required: which PPS the received label describes, `preceding` or `following` |
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
An exposure delayed beyond that window stays unmatched. A T interrupt sample
that arrives after Strobe also stays unmatched; no causal timestamp is fabricated.

CSV includes start/end/midpoint, `dur_us`, matched trigger time, latency, fit RMS,
anchor count, source sequences/ticks, flags, and the selected time-scale settings.
`dur_us` and latency use the same calibrated times as the timestamp columns.
Duration is blank unless both exposure endpoints have valid fits. CSV includes
`trigger_time_source`/`strobe_time_source`; the edge CSV includes `time_source`.
`latency_us` is an observed time difference containing ISR and path delays.

| Flag | Meaning |
|---|---|
| `0x01` | Incomplete time fit |
| `0x02` | No matched trigger time, or its fit failed |
| `0x04` | Suspect exposure reconstruction near a lost edge |
| `0x08` | Non-centered fit / incomplete window coverage |
| `0x10` | Missing exposure end edge |
| `0x40` | Log integrity incomplete or unverified |
| `0x80` | Matched Trigger pulse shortened on STOP |

Fit RMS describes residuals; it is not an absolute timing-accuracy guarantee.

## Validation

The implementation has passed 25 host regression tests, both Teensy 4.1 build
environments, and a C++20 host-client build with warnings treated as errors.
These are the results from the implementation verification; no firmware has been
flashed or board waveforms measured as part of that verification.

To repeat the software checks:

```bash
python3 -m unittest discover -s tests -v
pio run -e teensy41 -e teensy41_cli
g++ -std=c++20 -O2 -pthread -Wall -Wextra -Werror -DSESSION_CLIENT_DEMO session_client.cpp -o session_client
```

Host regressions run the real PWM driver and session command path against register
and GPIO models, including group validation, AUX clock dependence, divider
boundaries, merged/lost interrupts, queue overflow, truncated STOP, READY, LED
activity and post-processing. The serial integration tests exercise recorder
failures and acknowledgements. Models do not replace board measurements.

On the bench, first connect one camera at a time with its group enabled, then
connect both pairs. Verify PPS/NMEA arrival, exposure polarity and the camera's
accepted frame count. The remaining hardware acceptance checks are:

1. Measure FX at 20/25/50 Hz and the intended maximum operating rate; measure JAI
   at 1/2/5/10 Hz and around prescaler changes.
2. Test FX-only, JAI-only, both-on and both-off sessions. Compare JAI periods across
   sessions with different FX rates, keeping the JAI request fixed.
3. Measure period, pulse width and within-group edge skew at both the MCU pins and
   camera connectors, using the actual cables and camera load.
4. Repeat START/STOP, including STOP during a HIGH pulse. Confirm LOW idle outputs,
   no extra startup pulse and a matching `#TRUNC` record for a shortened pulse.
5. Run for at least 10 minutes with USB, NMEA, PPS and exposure traffic to cross
   multiple GPT rollovers. Compare independent logic-analyzer edge counts with the
   log, check integer-second time against a trusted reference, and measure ISR
   timestamp error before assigning an accuracy specification.

Check that D211-D216 are off while idle. During START, apply PPS and RX traffic
and exercise each camera channel separately: only its corresponding activity LEDs
should light. STOP must extinguish all six. Check D111-D113 against the actual
powered rails; these power LEDs are independent of START/STOP.

For **bare Teensy only**, disconnect external drivers before jumpering GPIO 33 to
GPIO 18 for a logic-level loopback. On the assembled board, the exposure buffer
already drives GPIO 18; do not bridge two outputs. Never connect a 9 V camera
trigger output directly to a Teensy pin. GPIO 41 is the schematic's 12 V sense
input, not an exposure input.

## Files

`SensorSync.ino` controls sessions and logging; `timebase.*` provides the GPT clock;
`channels.*` drives hardware PWM and captures strobes; `trigger_groups.h` shares
group parsing and timing calculations with the host; `timesync.*` queues PPS and
assembles NMEA; `leds.*` displays activity from their counters. `postprocess.py`
converts logs. `session_client.cpp` and `session_protocol.h` implement host recording;
`record_ctl.bash` manages its lifecycle. `upload.bash` builds/flashes; `tests/`
contains host regression tests.

Released under the [MIT License](LICENSE).
