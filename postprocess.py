#!/usr/bin/env python3
"""
postprocess.py - SensorSync log -> per-frame GNSS times (CSV).

  1. anchors: each NMEA time label (ZDA / RMC) + the tick of its PPS
     (--convention preceding: the PPS before the sentence, default; following: after)
  2. clock:   tick -> time by a linear fit of the anchors within +-window/2
  3. frames:  exposure = active edge -> next idle edge of a strobe channel;
              trigger  = latest trigger rising edge at most one period before it

Times are Unix seconds (UTC), or GPS seconds with --gps. Without PPS + NMEA,
times are seconds since the first logged edge at the nominal tick rate.
Requires numpy.
"""
import argparse
import calendar
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


GPS_EPOCH = 315964800  # 1980-01-06 in Unix seconds


def nmea_time(sentence):
    """Unix second labelled by a valid $--ZDA / $--RMC sentence on a whole second, else None."""
    body, _, checksum = sentence[1:].partition('*')
    if not sentence.startswith('$') or len(checksum) != 2:
        return None
    x = 0
    for c in body:
        x ^= ord(c)
    f = body.split(',')
    try:
        if int(checksum, 16) != x:
            return None
        if f[0].endswith('ZDA'):
            hms, day, month, year = f[1], int(f[2]), int(f[3]), int(f[4])
        elif f[0].endswith('RMC') and f[2] == 'A':
            hms, day, month, year = f[1], int(f[9][0:2]), int(f[9][2:4]), 2000 + int(f[9][4:6])
        else:
            return None
        if float(hms[6:] or 0) != 0:  # label must sit on the PPS second
            return None
        return calendar.timegm((year, month, day, int(hms[0:2]), int(hms[2:4]), int(hms[4:6])))
    except (ValueError, IndexError):
        return None


@dataclass
class Log:
    tick_hz: float = 75e6
    trig_hz: dict = field(default_factory=dict)   # ch -> Hz from #TRIG (0 = off)
    strobes: dict = field(default_factory=dict)   # ch -> (name, active_high) from #STROBE
    pps: list = field(default_factory=list)       # (seq, tick)
    tod: list = field(default_factory=list)       # (seq, tick, sentence)
    trig: dict = field(default_factory=dict)      # ch -> [(seq, level, tick)]
    strobe: dict = field(default_factory=dict)    # ch -> [(seq, level, tick)]


def read_log(path):
    """One Log per #SESSION,START (the whole file if there is none)."""
    logs = [Log()]
    with open(path, errors='replace') as fh:
        for line in fh:
            f = line.rstrip('\r\n').split(',')
            lg = logs[-1]
            try:
                if f[0] == '#SESSION' and f[1] == 'START' and (lg.pps or lg.tod or lg.trig or lg.strobe):
                    logs.append(Log())
                elif f[0] == '#LOG':
                    lg.tick_hz = next(float(x[8:]) for x in f if x.startswith('tick_hz='))
                elif f[0] == '#TRIG':
                    lg.trig_hz[int(f[1])] = float(f[4])
                elif f[0] == '#STROBE':
                    lg.strobes[int(f[1])] = (f[2], f[4] != 'ah=0' if len(f) > 4 else True)
                elif f[0] == 'P':
                    lg.pps.append((int(f[1]), int(f[2])))
                elif f[0] == 'Z':
                    lg.tod.append((int(f[1]), int(f[2]), line.rstrip('\r\n').split(',', 3)[3]))
                elif f[0] in ('T', 'S'):
                    store = lg.trig if f[0] == 'T' else lg.strobe
                    store.setdefault(int(f[2]), []).append((int(f[1]), int(f[3]), int(f[4])))
            except (ValueError, IndexError, StopIteration):
                pass  # torn line
    return logs


def lost(seqs):
    """Lines missing from a stream whose seq counts from 1."""
    return seqs[-1] - len(seqs) if seqs else 0


def anchors(lg, convention):
    """Sorted [pps_tick, unix_second] rows; labels disagreeing with the rest are dropped."""
    pps = np.array(sorted(t for _, t in lg.pps), dtype=np.int64)
    pairs = {}
    for _, tick, sentence in lg.tod:
        sec = nmea_time(sentence)
        i = int(np.searchsorted(pps, tick))
        if convention == 'preceding':
            i -= 1
        if sec is not None and 0 <= i < len(pps) and abs(tick - pps[i]) < lg.tick_hz:
            pairs[int(pps[i])] = sec
    a = np.array(sorted(pairs.items()), dtype=np.int64).reshape(-1, 2)
    if len(a) >= 3:  # a whole-second mislabel stands far off one global line
        x = (a[:, 0] - a[0, 0]) / lg.tick_hz
        y = (a[:, 1] - a[0, 1]).astype(float)
        a = a[np.abs(y - np.polyval(np.polyfit(x, y, 1), x)) < 0.5]
    return a


class Clock:
    """tick -> (whole, fraction) seconds; linear fit of the anchors within +-window/2."""

    def __init__(self, anchors, tick_hz, window):
        self.t, self.s = anchors[:, 0], anchors[:, 1]
        self.hz, self.half = tick_hz, window / 2 * tick_hz
        self.fits = {}

    def __call__(self, tick):
        i = int(np.searchsorted(self.t, tick))  # one fit per PPS interval
        if i not in self.fits:
            mid = (self.t[max(i - 1, 0)] + self.t[min(i, len(self.t) - 1)]) // 2
            dist = np.abs(self.t - mid)
            use = np.flatnonzero(dist <= self.half)
            if len(use) < 2:
                use = np.sort(np.argsort(dist)[:2])  # sparse: two nearest anchors
            t0, s0 = self.t[use[0]], self.s[use[0]]
            b, a = np.polyfit((self.t[use] - t0) / self.hz, self.s[use] - s0, 1)
            self.fits[i] = (t0, s0, a, b)
        t0, s0, a, b = self.fits[i]
        rel = a + b * (tick - t0) / self.hz  # small offsets keep ns precision
        whole = math.floor(rel)
        return int(s0) + whole, rel - whole


def shift(t, seconds):
    rel = t[1] + seconds
    whole = math.floor(rel)
    return t[0] + whole, rel - whole


def diff_us(a, b):
    return ((b[0] - a[0]) + (b[1] - a[1])) * 1e6


def stamp(t, offset):
    ns = round(t[1] * 1e9)
    return f'{t[0] + offset + ns // 10**9}.{ns % 10**9:09d}'


def exposures(edges, active_high):
    """[[start_seq, start_tick, end_tick or None]]: active edge -> next idle edge."""
    out = []
    for seq, level, tick in sorted(edges, key=lambda e: e[2]):
        if level == active_high:
            out.append([seq, tick, None])
        elif out and out[-1][2] is None:
            out[-1][2] = tick
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='SensorSync log -> per-frame GNSS times (CSV)')
    ap.add_argument('log')
    ap.add_argument('-o', '--out', help='CSV path (default: <log>.csv)')
    ap.add_argument('--convention', choices=['preceding', 'following'], default='preceding',
                    help='PPS an NMEA label belongs to (default: preceding)')
    ap.add_argument('--window', type=float, default=10.0, help='fit window, s (default 10)')
    ap.add_argument('--gps', action='store_true', help='GPS seconds instead of Unix UTC')
    ap.add_argument('--leap', type=int, default=18, help='GPS-UTC seconds for --gps (default 18)')
    ap.add_argument('--strobe-ah', default='', help='exposure polarity override, e.g. 2:0 (0 = active low)')
    ap.add_argument('--session', type=int, default=1, help='session number in a multi-session log')
    args = ap.parse_args(argv)

    logs = read_log(args.log)
    lg = logs[args.session - 1]
    if len(logs) > 1:
        print(f'{len(logs)} sessions in log; using session {args.session}', file=sys.stderr)
    streams = {'PPS': lg.pps, 'NMEA': lg.tod, **{f'T{c}': e for c, e in lg.trig.items()},
               **{f'S{c}': e for c, e in lg.strobe.items()}}
    for name, stream in streams.items():
        n = lost([e[0] for e in stream])
        if n:
            print(f'warning: {name} lost {n} line(s)', file=sys.stderr)
    for item in filter(None, args.strobe_ah.split(',')):
        ch, ah = map(int, item.split(':'))
        lg.strobes[ch] = (lg.strobes.get(ch, (f'S{ch}',))[0], bool(ah))

    a = anchors(lg, args.convention)
    if len(a) >= 2:
        scale, offset = ('gps', args.leap - GPS_EPOCH) if args.gps else ('utc', 0)
        print(f'{len(a)} PPS/NMEA anchors', file=sys.stderr)
    else:
        edges = [e[2] for store in (lg.trig, lg.strobe) for ev in store.values() for e in ev]
        if not edges:
            sys.exit('no events in log')
        t0 = min(edges)
        a = np.array([[t0, 0], [t0 + round(lg.tick_hz), 1]], dtype=np.int64)  # nominal rate
        scale, offset = 'rel', 0
        print('no PPS/NMEA anchors: times are seconds since the first edge', file=sys.stderr)
    clock = Clock(a, lg.tick_hz, args.window)

    out = Path(args.out or Path(args.log).with_suffix('.csv'))
    rows = 0
    with open(out, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['strobe', 'name', 'frame', f'start_{scale}', f'end_{scale}', f'mid_{scale}', 'dur_us',
                    f'trigger_{scale}', 'latency_us', 'start_seq', 'trigger_seq'])
        for ch, edges in sorted(lg.strobe.items()):
            name, active_high = lg.strobes.get(ch, (f'S{ch}', True))
            rises = sorted((tick, seq) for seq, level, tick in lg.trig.get(ch, []) if level == 1)
            ticks = np.array([t for t, _ in rises], dtype=np.int64)
            hz = lg.trig_hz.get(ch, 0)
            period = lg.tick_hz / hz if hz else 0
            for k, (seq, start_tick, end_tick) in enumerate(exposures(edges, active_high)):
                start = clock(start_tick)
                row = [ch, name, k, stamp(start, offset), '', '', '', '', '', seq, '']
                if end_tick is not None:
                    end = clock(end_tick)
                    dur = diff_us(start, end)
                    row[4:7] = [stamp(end, offset), stamp(shift(start, dur / 2e6), offset), f'{dur:.3f}']
                i = int(np.searchsorted(ticks, start_tick, 'right')) - 1
                if i >= 0 and start_tick - ticks[i] <= period:
                    trig = clock(ticks[i])
                    row[7:9] = [stamp(trig, offset), f'{diff_us(trig, start):.3f}']
                    row[10] = rises[i][1]
                w.writerow(row)
                rows += 1
    print(f'wrote {rows} frames -> {out}', file=sys.stderr)


if __name__ == '__main__':
    main()
