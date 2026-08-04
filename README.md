# SensorSync-Logger

**Raw event logger for post-processed GNSS timing — Teensy 4.1.**

![platform](https://img.shields.io/badge/platform-Teensy%204.1-blue)
![framework](https://img.shields.io/badge/framework-Arduino%20%7C%20PlatformIO-00979D)
![post](https://img.shields.io/badge/post--processing-Python%20%2B%20NumPy-informational)
![license](https://img.shields.io/badge/license-MIT-green)

SensorSync-Logger takes an INS's **PPS** and **NMEA time-of-day**, fires camera **trigger pulses**, reads back each camera's **exposure-active / strobe** signal, and **logs raw ticks + raw NMEA** to the host. It computes **no GNSS time on the board**. A host script pairs ToD↔PPS and does a **centered** sliding-window least-squares fit to convert every logged tick to GNSS time offline.

```
INS ──PPS───────────► Teensy 4.1  (GPIO IRQ, or GPT1 hardware capture)
    └─NMEA (ZDA/RMC)─► Serial2                 [logged raw, parsed offline]
                          │
      Teensy 4.1 ─────────┤
                          ├─ trig[0..2]   ──► level shift ──► camera Trigger In   (plain timer)
                          ├─ strobe[0..2] ◄── level shift ◄── camera Exposure Active
                          └─ USB          ──► raw event log ──► host ──► postprocess.py ──► frames.csv
```

## Why log raw and post-process?

The board only needs to **know** when each frame was triggered and exposed — it does **not** need to fire triggers at exact GPS instants. That makes the on-board real-time GNSS fit unnecessary, and moving the timestamping offline is both **simpler** and **more robust/accurate**:

- **No causal extrapolation bias.** A real-time on-board fit can only use *past* PPS, so its estimate at "now" is a trailing extrapolation whose bias tracks any oscillator frequency curvature (thermal drift) and appears in no statistic. Offline, each event is at the **center** of its fit window (PPS on both sides), so the fit interpolates — the bias cancels to first order.
- **Re-runnable & tunable.** Window length, robust estimator, PPS↔ToD convention, leap handling, outlier rejection — all chosen offline with full hindsight and human QC, and re-run if you find a bug. The lock/holdover/whole-second-mislabel machinery a real-time fit needs simply doesn't exist here.
- **Same measurements.** Both approaches use the identical raw ticks; only the fit changes. There is no accuracy loss for the offline approach when you don't need real-time answers.

The one thing post-processing can't recover is a **dropped sample**, so every logged stream carries a sequence number (gaps are detectable) and the board **never blocks `loop()`** (drops are counted, never spun on). A lightweight on-board health readout lets you catch a dead PPS / ToD during acquisition instead of at post-time.

## How it works

The board runs a free-running **GPT1** counter (75 MHz, 13.3 ns/tick, software-extended to 64 bit) as its only ruler. It logs:

| Line | Meaning |
|---|---|
| `P,<seq>,<tick>` | a PPS edge |
| `T,<seq>,<ch>,<level>,<tick>` | a trigger edge (`tick` = the exact GPT1 compare value) |
| `S,<seq>,<ch>,<level>,<tick>` | a strobe / exposure-active edge |
| `Z,<seq>,<tick>,<raw NMEA>` | a ToD sentence + the tick it was read out |

plus `#LOG` / `#TRIG` / `#STROBE` header lines (so the log is self-describing — re-emitted on the `h` command, which the recorder sends on connect, so a mid-session capture still gets them) and `#H` health lines every 5 s. `<seq>` is per-stream and advances even on a drop, so any gap is visible. Triggers free-run off a plain periodic GPT1 compare (not GNSS-aligned); the exact compare tick of each edge is logged, so the host still derives their precise GNSS time.

`postprocess.py` then: parses the log → parses each NMEA sentence and pairs it with a PPS edge → builds `(pps_tick, gnss_second)` anchors → for every trigger/strobe tick does a **centered robust linear fit** `tick → GNSS second` → reconstructs pulses and exposure windows, matches frames, and writes a per-frame CSV.

## Hardware

| Signal | Teensy pin | Notes |
|---|---|---|
| PPS in | 2 (or **48** for hardware capture) | rising edge |
| NMEA in | 7 (RX2) | 115200; **RS-232 needs a MAX3232**, not a divider |
| trig[0] `FX10E` | 29 | 1 Hz (plain timer; per-channel rate in `config.h`) |
| trig[1] `JAI` | 30 | 5 Hz |
| trig[2] `SPARE` | 31 | 1 Hz |
| strobe[0..2] | 32 / 33 / 34 | exposure-active read-back |

- **Teensy 4.1 is 3.3 V and not 5 V tolerant.** Level-shift every camera/INS line.
- Level shifters add a few µs of delay, asymmetric on rise vs fall — measure them and correct in post-processing (the exposure edges carry that delay).
- Strobe inputs are biased toward the inactive level, so an unpowered line reads "no exposure" instead of flooding the log with noise.

## Build, flash & record

[PlatformIO](https://platformio.org/):

```bash
pio run                 # build
pio run -t upload       # flash
./record.bash           # record (interactive): auto-detect the port, one file per (re)connect
```

Two recorders (both bash, no dependencies; both send the `h` command on connect so every file starts with the `#LOG/#TRIG/#STROBE` header, and both keep the baud at **115200** — leave it):

- **Interactive** — `./record.bash [outdir]` : auto-detects the Teensy, writes `logs/session_<timestamp>.log`, and starts a new file on every reset/replug. Good for a manual run.
- **Driver-controlled** — `./record_ctl.bash {start [outdir] | stop | new | status}` : call from your acquisition software so **each acquisition run = one new `logs/session_<timestamp>.log`**. `start` prints the log path to stdout and returns at once; `stop` ends it; `new` rotates (stop + start). See *Field acquisition* below.

Don't run a recorder together with `pio device monitor` (one reader per port). Minimal alternative: `pio device monitor -b 115200 > session.log`.

**Console commands** (single characters over USB): `s` health · `r` reset stats · `a` enable triggers · `d` disable triggers · `h` reprint the header.

### Field acquisition (one log per run)

Drive `record_ctl.bash` from your sensor-acquisition software so every acquisition gets its own self-contained log:

```bash
LOG=$(./record_ctl.bash start /data/logs)   # begin a run; capture the path it prints
# ... run the acquisition ...
./record_ctl.bash stop                        # end the run
# later, offline:
python3 postprocess.py "$LOG" -o "${LOG%.log}.csv" --gps
```

Or from Python:

```python
import subprocess
log = subprocess.run(["./record_ctl.bash", "start", "/data/logs"],
                     capture_output=True, text=True).stdout.strip()
# ... acquire ...
subprocess.run(["./record_ctl.bash", "stop"])
```

For a truly self-contained run (sequence numbers from 1, fresh header), press the Teensy **reset** just before `start` — otherwise the counters simply continue, which post-processing handles fine either way. Within a run a brief USB glitch reconnects and appends to the **same** file.

**Linux:** install the PJRC udev rules or ModemManager will hold `/dev/ttyACM0` (`sudo wget -O /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules && sudo udevadm control --reload-rules && sudo udevadm trigger`, then replug).

> Capture with a tool that reads continuously. If the host stalls, the board drops and counts (`host=` in `#H`) — a `seq` gap in the log marks exactly what was lost.

## Configuration

Everything hardware-specific is in [`config.h`](config.h):

| Setting | Default | Meaning |
|---|---|---|
| `TB_PERCLK_DIV` | 2 | GPT clock divider → 75 MHz, 13.3 ns/tick |
| `PPS_PIN` / `PPS_RISING_EDGE` | 2 / 1 | PPS pin & active edge (48 for hardware capture) |
| `TOD_INPUT_SERIAL` / `TOD_BAUD` | Serial2 / 115200 | NMEA input |
| `TRIG_CFG[]` | — | per channel: name, pin, freq (≥1 Hz), phase, pulse width, active_high |
| `STROBE_CFG[]` | — | per channel: name, pin, active_high |
| `USE_HW_CAPTURE` | 0 | PPS via `GPT1_CAPTURE1` (needs PPS on pin 48); the main PPS-jitter lever |
| `EVT_RING_SIZE` / `DRAIN_PER_LOOP` | 4096 / 96 | ISR→loop event ring / max lines emitted per stream per loop |

There is **no** time-scale / GPS-offset / preceding-vs-following / lag-window / lock / holdover config on the board — all of that is decided offline by `postprocess.py`.

### PPS hardware capture (`USE_HW_CAPTURE`)

Optional, and the single biggest data-quality lever (PPS jitter is what most limits the fit). Latches the PPS edge in `GPT1_CAPTURE1` instead of a GPIO ISR, dropping its timestamp jitter to one tick. Requires **PPS on pin 48** (`GPIO_EMC_24`, ALT4 — the only usable capture pad on Teensy 4.1; a `static_assert` enforces it). Bring-up: jumper `HW_SELFTEST_DRIVE_PIN` (31) → 48, set `USE_HW_CAPTURE 1` / `PPS_PIN 48`, leave `HW_PPS_DAISY` at `AUTO`; the self-test prints the working daisy value and **refuses to log fake PPS on failure**; hard-code that value, remove the jumper, set `HW_CAPTURE_SELFTEST 0`.

## Post-processing

Needs only NumPy.

```bash
python3 postprocess.py session.log -o frames.csv          # UTC/Unix seconds
python3 postprocess.py session.log -o frames.csv --gps    # GPS seconds
python3 postprocess.py session.log --window 10 --convention auto --edges edges.csv
```

Options: `--window S` centered fit window (default 10 s), `--convention auto|preceding|following` (default `auto` = nearest PPS; it prints the median ToD→PPS lag so you can sanity-check), `--gps`/`--leap N` GPS-second output, `--link 0:0,1:1` strobe→trigger channel map (default same index), `--edges FILE` also dump every edge's GNSS time.

**Per-frame CSV** (`frames.csv`): `strobe,name,frame,start_sec,start_ns,end_sec,end_ns,mid_sec,mid_ns,dur_us,trig_sec,trig_ns,latency_us,fit_rms_ns,n_anchors,flags`. Exposure midpoint and duration are computed; `latency_us` = exposure start − matched trigger command. `flags`: `0x02` no trigger matched, `0x04` exposure suspect (an edge was lost mid-window), `0x08` non-centered fit (within a fit-window of the session start/end, or next to a gap — trust it less).

The script also prints, to stderr, dropped-edge counts (from `seq` gaps) and the anchor / lag summary.

## Bring-up & validation

1. **Bare board** — flash, capture the log, confirm the `#LOG`/`#TRIG`/`#STROBE` header and 5 s `#H` lines; `pps=` should be 0 with no INS.
2. **INS connected** — `pps=` climbs ~1/s, `tod=` climbs ~1/s, `P`/`Z` lines appear; `T`/`S` lines stream at the trigger/exposure rate.
3. **First post-process** — run `postprocess.py`; check it reports a sane anchor count and median lag (100–400 ms is a typical `preceding` receiver), and that per-frame times look right.
4. **Integer-second check (most error-prone)** — compare a frame's `start_sec` to trusted time. Off by exactly 1 s ⇒ force `--convention` the other way (this is the fundamental PPS + whole-second-NMEA ambiguity; the log can't resolve it alone, so decide it here).
5. **Loopback** — jumper `trig[2]` (31) → `strobe[2]` (34); `latency_us` is then the full command-to-measurement path. Repeat on the highest-rate channel (JAI) too.
6. **Gap / stall test** — pause the capturing host briefly; confirm the log shows a `seq` gap and `#H host=` climbs, and that `postprocess.py` reports the dropped edges rather than silently mis-timing.

## Known limitations

- Trigger frequency must be **≥ 1 Hz** (period is a whole number of ticks); sub-1-Hz is rejected loudly at `begin()`. Triggers are **not** GNSS-aligned — only their logged ticks are precise.
- Strobe (and PPS, in software mode) timestamps carry GPIO-ISR jitter (~0.3–2 µs). PPS hardware capture removes it for PPS; strobe jitter is the floor for per-frame timing unless the camera's exposure edge is itself hardware-captured.
- The PPS↔whole-second-NMEA pairing has a fundamental ±1 s ambiguity that no amount of logging resolves; `postprocess.py --convention` decides it (defaulting to nearest-PPS, correct when receiver latency < 0.5 s).
- Post-processing needs PPS on **both sides** of an event for a centered fit, so the first/last fit-window of a session is one-sided (flagged `0x08`).

## Files

| File | Role |
|---|---|
| `SensorSync.ino` | setup/loop, raw-log output, health, console |
| `timebase.*` | GPT1 free-running counter (64-bit) + optional PPS hardware capture |
| `timesync.*` | PPS tick queue + raw NMEA assembly + health (no fit) |
| `channels.*` | plain-timer trigger generation + strobe capture + event rings |
| `config.h` | all hardware-specific settings |
| `postprocess.py` | offline ToD↔PPS pairing + centered fit + per-frame CSV |
| `record.bash` | interactive capture: auto-detect port, timestamped file, header on connect, auto-reconnect |
| `record_ctl.bash` | programmatic `start`/`stop`/`new`/`status` — one log file per acquisition run |

> A GNSS-*disciplined* variant of this firmware (real-time on-board timestamps, GPS-aligned triggers, GPT2 hardware compare, and a full Chinese code walkthrough) lives on the **`main`** branch.

## License

Released under the [MIT License](LICENSE).
