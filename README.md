# SensorSync-Logger

Teensy 4.1 event logger for GNSS-timed cameras (Specim FX10e, JAI Go-X).
The board stamps every PPS, trigger and exposure edge with one 75 MHz counter and forwards the raw NMEA.
`postprocess.py` then converts the ticks to UTC or GPS time for each frame.
Triggers free-run and are not aligned to GNSS.

```text
INS PPS + NMEA, camera exposure ──> Teensy 4.1 ── USB ──> raw log ──> postprocess.py ──> CSV
                                        └──> camera triggers (hardware PWM)
```

## Quick start

```bash
./upload.bash                     # build + flash firmware (-b: build only)
g++ -std=c++17 -O2 -pthread -DSESSION_CLIENT_DEMO session_client.cpp -o session_client

LOG=$(./record_ctl.bash start /data/logs FX:22.5,JAI:6)   # record in the background
./record_ctl.bash stop                                    # nonzero exit if the run failed

python3 postprocess.py "$LOG"     # -> ${LOG%.log}.csv
```

## Hardware

Pins are Arduino GPIO numbers ([schematic](resource/SCH_Schematic1_2026-08-26.pdf)).

| Ch | Camera  | Trigger | Exposure | Exposure level   | Trigger output |
|----|---------|--------:|---------:|------------------|----------------|
| 0  | FX10E-1 |      33 |       18 | high             | on             |
| 1  | FX10E-2 |       4 |       10 | high             | off            |
| 2  | JAI-1   |      37 |       14 | low              | on             |
| 3  | JAI-2   |      36 |       15 | low (unverified) | off            |

PPS: GPIO 19, rising edge.
NMEA: Serial1 RX (GPIO 0), 115200 8N1.

All of this is set in `config.h`:

- Channels 0/1 share one PWM rate (Specim FX10e pair), and channels 2/3 share another (JAI Go-X pair).
- `TRIG_CFG[].enabled = false` keeps a trigger pin LOW and out of the log.
- `STROBE_CFG[].active_high` is the line level during exposure. Check it for each camera model.

Teensy GPIO accepts 3.3 V only.
RS-232 must pass through the MAX3232, and the 9 V trigger outputs must never touch a Teensy pin.
Check the trigger pulse at the camera connector.

## Recording

```bash
./session_client <port> <log> [seconds, 0 = until Ctrl-C] [FX:<hz>,JAI:<hz>]
./record_ctl.bash start [outdir] [rates] | stop | status   # background wrapper
```

`record_ctl.bash` uses the first Teensy port. Set `SENSOR_PORT` to choose a different one.

From C++, include `session_client.cpp`:

```cpp
SensorSyncSession s;
if (!s.open(port) || !s.start(log_path, {{0, 22.5}, {2, 6.0}})) {  // channel -> Hz
    fail(s.lastError());
}
// acquisition; s.ok() turns false on a port/file error, #ERR or reported drops
s.stop();
```

`open()` sends STOP, which ends any session left running.
Only one program may use the port at a time.

## Log format

Board commands: `START [freq=FX:<hz>,JAI:<hz>] [path]`, `STOP`, `s` (status).
A rate of 0 turns a pair off, and an omitted pair keeps its last rate.
A rate the hardware cannot produce (for example Specim FX10e below ~18 Hz) gets `#ERR,bad_freq`.

| Line                                             | Meaning                                  |
|--------------------------------------------------|------------------------------------------|
| `#SESSION,START,<path>` … `#SESSION,STOP,<path>` | One recording                            |
| `#LOG,SensorSync-logger,3,tick_hz=<f>`           | Format version, nominal tick rate        |
| `#GROUP,<FX\|JAI>,requested=,actual=,pulse_us=`  | Pair rate after hardware quantization    |
| `#TRIG,<ch>,<name>,<pin>,<hz>`                   | Trigger output; 0 Hz = off               |
| `#STROBE,<ch>,<name>,<pin>,ah=<0\|1>`            | Exposure input and its active level      |
| `#READY`                                         | Triggers running                         |
| `P,<seq>,<tick>`                                 | PPS edge                                 |
| `T,<seq>,<ch>,1,<tick>`                          | Trigger rising edge                      |
| `S,<seq>,<ch>,<level>,<tick>`                    | Exposure edge                            |
| `Z,<seq>,<tick>,<NMEA>`                          | NMEA sentence; tick when read (1 ms poll)|
| `#H … drops=<n>`                                 | Counts, every 5 s and at STOP            |

`seq` counts from 1 in each stream, so a missing line shows up as a gap.
`drops` counts lines lost to a full queue or a full USB link.
After STOP, the board keeps capturing exposures for 200 ms (`STOP_TAIL_MS`) so that the last frame closes.

## Post-processing

`python3 postprocess.py <log> [options]` writes one CSV row per exposure.

- **Anchors:** each ZDA/RMC time label is paired with the PPS before it.
This is correct for the AsteRx-RBi3;
use `--convention following` for a receiver that sends the label before its PPS.
- **Clock fit:** ticks become time through a linear fit over the PPS within ±`--window`/2 s (default 10 s).
A label that is off by whole seconds is dropped.
- **Exposures:** an exposure runs from an active edge to the next idle edge.
Its trigger is the latest rising edge on the same channel within one trigger period before it.
- **No GNSS:** without PPS/NMEA, times are seconds since the first edge.

| Option              | Use                                                         |
|---------------------|-------------------------------------------------------------|
| `-o FILE`           | Output path (default `<log>.csv`)                           |
| `--gps [--leap 18]` | GPS seconds instead of Unix UTC                             |
| `--strobe-ah 2:0`   | Override an exposure level, e.g. old logs with JAI `ah=1`   |
| `--session N`       | Pick one session from a multi-session log                   |

CSV columns: `strobe, name, frame, start_*, end_*, mid_*, dur_us, trigger_*, latency_us, start_seq, trigger_seq`, where `*` is `utc`, `gps` or `rel`.

A tick (13.3 ns) is the resolution, not the accuracy.
Interrupt latency and cabling add fixed path delays that the PPS fit does not remove.
Trigger matching is a timing heuristic, not a camera frame ID.

## Development

```bash
python3 -m unittest discover -s tests -v   # post-processing tests
```

| File                                    | Role                                          |
|-----------------------------------------|-----------------------------------------------|
| `SensorSync.ino`                        | Commands, log output                          |
| `config.h`                              | Pins, rates, exposure levels                  |
| `events.*`                              | ISR timestamps → event queue                  |
| `triggers.*`                            | FlexPWM trigger pulses                        |
| `timebase.*`                            | 64-bit GPT1 clock (do not use `IntervalTimer`)|
| `leds.*`                                | Activity LEDs                                 |
| `session_client.cpp`, `record_ctl.bash` | Host recording                                |
| `postprocess.py`                        | Per-frame GNSS times                          |

Released under the [MIT License](LICENSE).
