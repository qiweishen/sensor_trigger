#!/usr/bin/env python3
"""
postprocess.py - turn a SensorSync-Logger raw log into per-frame GNSS times.

The board logs only raw ticks + raw NMEA (it computes no GNSS time). This script:
  1. parses the log (#LOG/#TRIG/#STROBE header + P/T/S/Z lines),
  2. parses each NMEA sentence and pairs it using an operator-verified PPS convention,
  3. builds (pps_tick, gnss_second) anchors and, for every trigger/strobe edge,
     does a CENTERED sliding-window robust linear fit tick->GNSS time -- which
     removes the causal extrapolation bias of an on-board trailing fit,
  4. reconstructs trigger pulses and exposure windows, matches frames, and writes
     a per-frame CSV (and optionally a per-edge CSV).

Only dependency: numpy.

Usage requires explicitly verified --tod-scale (utc/gps) and --convention
(preceding/following). GPS conversion also requires the applicable --leap.
Output CSV rows describe strobe observations; they are not camera BlockIDs.
Current T ticks are samples immediately after GPIO writes; older logs use scheduled
compares (see #TIME). S and P ticks are GPIO ISR samples. Fits do not remove path latency.
"""

import argparse
import csv
import sys
from datetime import date
from dataclasses import dataclass, field

import numpy as np

GPS_EPOCH_UNIX = 315964800  # 1980-01-06 00:00:00 UTC, in Unix seconds


# ---------------------------------------------------------------------------
# NMEA parsing (RMC / ZDA -> Unix whole second)
# ---------------------------------------------------------------------------
def _days_from_civil(y, m, d):
    # Howard Hinnant's algorithm (days since 1970-01-01).
    y -= 1 if m <= 2 else 0
    era = (y if y >= 0 else y - 399) // 400
    yoe = y - era * 400
    doy = (153 * (m + (-3 if m > 2 else 9)) + 2) // 5 + d - 1
    doe = yoe * 365 + yoe // 4 - yoe // 100 + doy
    return era * 146097 + doe - 719468


def _checksum_ok(s):
    if len(s) < 4 or s[0] != '$':
        return False
    star = s.rfind('*')
    if star < 0 or star + 3 != len(s):
        return False
    try:
        want = int(s[star + 1:star + 3], 16)
    except ValueError:
        return False
    cs = 0
    for ch in s[1:star]:
        cs ^= ord(ch)
    return cs == want


def parse_nmea_unix(s):
    """Return the UTC Unix whole second a $--RMC / $--ZDA sentence describes, or None."""
    if not _checksum_ok(s):
        return None
    body = s[1:s.rfind('*')]
    f = body.split(',')
    typ = f[0][2:] if len(f[0]) >= 5 else f[0]
    try:
        if typ == 'RMC':
            # $--RMC,hhmmss.ss,A,lat,N,lon,E,spd,cog,ddmmyy,...
            if len(f) < 10 or f[2] != 'A':
                return None
            tf, df = f[1], f[9]
            if len(tf) < 6 or len(df) < 6:
                return None
            hh, mm, ss = int(tf[0:2]), int(tf[2:4]), int(tf[4:6])
            dd, mo, yy = int(df[0:2]), int(df[2:4]), 2000 + int(df[4:6])
            frac = tf[6:]
        elif typ == 'ZDA':
            # $--ZDA,hhmmss.ss,dd,mm,yyyy,zz,zz
            if len(f) < 5:
                return None
            tf = f[1]
            if len(tf) < 6:
                return None
            hh, mm, ss = int(tf[0:2]), int(tf[2:4]), int(tf[4:6])
            dd, mo, yy = int(f[2]), int(f[3]), int(f[4])
            frac = tf[6:]
        else:
            return None
    except ValueError:
        return None
    # Must sit on the PPS second (no sub-second label), and not a leap second.
    if frac and frac.lstrip('.').strip('0') != '':
        return None
    if ss == 60:
        return None
    if not (0 <= hh <= 23 and 0 <= mm <= 59 and 0 <= ss <= 59
            and 1 <= dd <= 31 and 1 <= mo <= 12 and 2000 <= yy <= 2199):
        return None
    try:
        date(yy, mo, dd)
    except ValueError:
        return None
    return _days_from_civil(yy, mo, dd) * 86400 + hh * 3600 + mm * 60 + ss


# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------
@dataclass
class Log:
    tick_hz: float = 75_000_000.0
    trig_cfg: dict = field(default_factory=dict)    # ch -> {name, pin, freq, active_high}
    strobe_cfg: dict = field(default_factory=dict)  # ch -> {name, pin, active_high}
    pps: list = field(default_factory=list)         # (seq, tick)
    trig: dict = field(default_factory=dict)        # ch -> [(seq, level, tick)]
    strobe: dict = field(default_factory=dict)      # ch -> [(seq, level, tick)]
    tod: list = field(default_factory=list)         # (seq, tick, unix_sec) valid only
    tod_bad: int = 0
    tod_seq: list = field(default_factory=list)     # every Z seq (good or bad NMEA)
    from_marker: bool = False                       # began at #SESSION,START
    session_path: str = ''
    first_line: int = 1
    last_line: int = 0
    malformed: int = 0
    stopped: bool = False
    final_health: bool = False
    integrity_issues: set = field(default_factory=set)


def _has_data(lg):
    return bool(lg.pps or lg.tod_seq or lg.malformed
                or any(lg.trig.values()) or any(lg.strobe.values()))


def parse_log(path):
    """Return list of session Logs: one per #SESSION,START (whole file if no markers)."""
    sessions = []
    cur = Log()
    fh = sys.stdin if path == '-' else open(path, 'r', errors='replace')
    with fh:
        for lineno, line in enumerate(fh, 1):
            line = line.rstrip('\r\n')
            if not line:
                continue
            if line.startswith('#SESSION,START'):
                p = line.split(',', 2)
                spath = p[2] if len(p) > 2 else ''
                if cur.from_marker or _has_data(cur):
                    # new session bucket; carry config forward (headers may repeat)
                    sessions.append(cur)
                    nxt = Log(tick_hz=cur.tick_hz, from_marker=True,
                              session_path=spath, first_line=lineno)
                    nxt.trig_cfg = {k: dict(v) for k, v in cur.trig_cfg.items()}
                    nxt.strobe_cfg = {k: dict(v) for k, v in cur.strobe_cfg.items()}
                    for ch in nxt.trig_cfg:
                        nxt.trig[ch] = []
                    for ch in nxt.strobe_cfg:
                        nxt.strobe[ch] = []
                    cur = nxt
                else:
                    # header-only prefix: absorb marker into current bucket
                    cur.from_marker = True
                    cur.session_path = spath
                cur.last_line = lineno
                continue
            cur.last_line = lineno
            if line.startswith('#'):
                try:  # torn header line (USB glitch boundary) must not kill the run
                    if line.startswith('#SESSION,STOP,'):
                        cur.stopped = True
                        if not cur.final_health:
                            cur.integrity_issues.add('STOP without final health')
                    elif line.startswith('#ERR'):
                        cur.integrity_issues.add(line)
                    elif line.startswith(('#H ', '#Ht ', '#HFINAL ')):
                        final = line.startswith('#HFINAL ')
                        cur.final_health |= final
                        fields = dict(tok.split('=', 1) for tok in line.split() if '=' in tok)
                        for key in ('ppsdrops', 'todtrunc', 'tdrops', 'sdrops', 'host', 'skip', 'stall'):
                            if key in fields and int(fields[key]) != 0:
                                cur.integrity_issues.add(f'{key}={fields[key]}')
                            if final and key not in fields:
                                cur.integrity_issues.add(f'final health missing {key}')
                    elif line.startswith('#LOG'):
                        for tok in line.split(','):
                            if tok.startswith('tick_hz='):
                                cur.tick_hz = float(tok[8:])
                            elif tok == 'cap=0':
                                cur.integrity_issues.add('PPS hardware capture unavailable')
                    elif line.startswith('#TRIG,'):
                        p = line.split(',')
                        ch = int(p[1])
                        ah = int(p[5].split('=')[1]) if len(p) > 5 and '=' in p[5] else 1
                        cur.trig_cfg[ch] = dict(name=p[2], pin=int(p[3]), freq=float(p[4]), active_high=ah)
                        cur.trig.setdefault(ch, [])
                    elif line.startswith('#STROBE,'):
                        p = line.split(',')
                        ch = int(p[1])
                        ah = int(p[4].split('=')[1]) if len(p) > 4 and '=' in p[4] else 1
                        cur.strobe_cfg[ch] = dict(name=p[2], pin=int(p[3]), active_high=ah)
                        cur.strobe.setdefault(ch, [])
                except (ValueError, IndexError):
                    cur.malformed += 1
                continue
            t = line[0]
            p = line.split(',')
            try:
                if t == 'P':
                    cur.pps.append((int(p[1]), int(p[2])))
                elif t == 'T':
                    cur.trig.setdefault(int(p[2]), []).append((int(p[1]), int(p[3]), int(p[4])))
                elif t == 'S':
                    cur.strobe.setdefault(int(p[2]), []).append((int(p[1]), int(p[3]), int(p[4])))
                elif t == 'Z':
                    # Z,<seq>,<tick>,<raw nmea with its own commas>
                    seq = int(p[1]); tick = int(p[2])
                    nmea = line.split(',', 3)[3]
                    cur.tod_seq.append(seq)
                    sec = parse_nmea_unix(nmea)
                    if sec is None:
                        cur.tod_bad += 1
                    else:
                        cur.tod.append((seq, tick, sec))
            except (ValueError, IndexError):
                cur.malformed += 1
                continue
    sessions.append(cur)
    return sessions


def report_gaps(name, seqs, from_start=False):
    if not seqs:
        return 0
    a = np.asarray(seqs, dtype=np.int64)
    if np.any(a < 1) or (len(a) > 1 and np.any(np.diff(a) <= 0)):
        raise ValueError(f"{name}: duplicate/reversed sequence or counter reset; split clock epochs before fitting")
    # marker sessions: firmware guarantees seq starts at 1 -> leading drops countable
    lost = int(a[0] - 1) if from_start and a[0] > 1 else 0
    if len(a) > 1:
        d = np.diff(a)
        lost += int(np.sum(d[d > 1] - 1))
    if lost:
        print(f"  ! {name}: {lost} dropped edge(s) (seq gaps) across {len(a)} logged", file=sys.stderr)
    return lost


# ---------------------------------------------------------------------------
# ToD <-> PPS pairing
# ---------------------------------------------------------------------------
def build_anchors(lg, convention, tol_ppm=200.0, verbose=True):
    """Return sorted anchors ndarray [[pps_tick, unix_sec], ...] and the median lag (s)."""
    if convention not in ('preceding', 'following'):
        raise ValueError("PPS convention must be explicitly preceding or following")
    if not lg.pps or not lg.tod:
        return np.empty((0, 2)), 0.0
    pps_ticks = np.array(sorted(t for _, t in lg.pps), dtype=np.int64)
    hz = lg.tick_hz
    anchors = {}   # pps_tick -> unix_sec
    lags = []
    for _, z_tick, sec in lg.tod:
        i = np.searchsorted(pps_ticks, z_tick)
        before = pps_ticks[i - 1] if i > 0 else None
        after = pps_ticks[i] if i < len(pps_ticks) else None
        if convention == 'preceding':
            cand = before
        else:
            cand = after
        if cand is None:
            continue
        lag = (z_tick - cand) / hz          # + => sentence after its PPS (preceding convention)
        if abs(lag) > 1.5:                  # no PPS anywhere near this sentence
            continue
        lags.append(lag)
        if int(cand) in anchors and anchors[int(cand)] != int(sec):
            raise ValueError("Conflicting ToD labels for one PPS; verify the pairing convention")
        anchors[int(cand)] = int(sec)
    if not anchors:
        return np.empty((0, 2)), 0.0
    arr = np.array(sorted(anchors.items()), dtype=np.int64)   # [[tick, sec], ...]
    med_lag = float(np.median(lags)) if lags else 0.0

    # Reject implausible anchors: ref rate = median over adjacent pairs, chain vs last KEPT.
    if len(arr) >= 2:
        dt = np.diff(arr[:, 0]).astype(np.float64)
        ds = np.diff(arr[:, 1]).astype(np.float64)
        fwd = ds > 0
        ref = float(np.median(dt[fwd] / ds[fwd])) if fwd.any() else hz
        # self-derived ref must stay within crystal territory of nominal, else small /
        # corrupt anchor sets self-validate (a 1 s mislabel shifts the rate ~100%)
        if abs(ref - hz) / hz > 500e-6:
            print(f"  ! anchor rate {ref:.0f} deviates from nominal {hz:.0f} by "
                  f"{abs(ref-hz)/hz*1e6:.0f} ppm; using nominal for plausibility", file=sys.stderr)
            ref = hz

        def _plaus(i, j):
            dsec = int(arr[j, 1] - arr[i, 1])
            if dsec <= 0:
                return False
            return abs((arr[j, 0] - arr[i, 0]) / dsec - ref) / ref * 1e6 <= tol_ppm

        # seed = first anchor consistent with a near successor (catches bad FIRST anchor)
        seed = None
        for i in range(len(arr) - 1):
            if _plaus(i, i + 1) or (i + 2 < len(arr) and _plaus(i, i + 2)):
                seed = i
                break
        if seed is None:
            seed = len(arr) - 1
        keep = np.zeros(len(arr), dtype=bool)
        keep[seed] = True
        last = seed
        for i in range(seed + 1, len(arr)):
            if _plaus(last, i):
                keep[i] = True
                last = i
        arr = arr[keep]

    if verbose:
        print(f"  anchors: {len(arr)} PPS<->ToD pairs, median lag {med_lag*1e3:.1f} ms "
              f"({'preceding' if med_lag >= 0 else 'following'} side)", file=sys.stderr)
    return arr, med_lag


# ---------------------------------------------------------------------------
# Centered sliding-window robust fit: tick -> GNSS second
# ---------------------------------------------------------------------------
class CenteredFitter:
    def __init__(self, anchors, tick_hz, window_s=10.0, min_pts=4):
        self.at = anchors[:, 0].astype(np.int64)         # pps ticks (sorted)
        self.asec = anchors[:, 1].astype(np.int64)       # unix seconds
        self.hz = tick_hz
        self.half = int(window_s * tick_hz / 2)
        self.min_pts = min_pts

    def _fit(self, xt, ys):
        # Robust least squares (2 passes, 3-sigma rejection). Work in small offsets.
        t0 = xt[0]
        s0 = ys[0]
        x = (xt - t0).astype(np.float64)
        y = (ys - s0).astype(np.float64)
        kept = xt.copy()
        floor = 5.0 / self.hz                    # ~5 ticks: don't reject on mere quantization
        for _ in range(2):
            if len(x) < 4:
                break
            b, a = np.polyfit(x, y, 1)           # y = a + b*x
            r = y - (a + b * x)
            thr = max(3.0 * np.std(r), 3.0 * floor)
            m = np.abs(r) <= thr
            if m.all() or m.sum() < 4:
                break
            x, y = x[m], y[m]
            kept = kept[m]
        b, a = np.polyfit(x, y, 1)
        r = y - (a + b * x)
        rms = float(np.sqrt(np.mean(r * r))) if len(r) else 0.0
        return a, b, t0, s0, rms, kept

    def gnss(self, tick):
        """Return (whole_sec_int, frac_float, rms_residual_s, n_anchors, ok, centered)."""
        lo = np.searchsorted(self.at, tick - self.half, 'left')
        hi = np.searchsorted(self.at, tick + self.half, 'right')
        xt, ys = self.at[lo:hi], self.asec[lo:hi]
        centered = (len(xt) >= self.min_pts and
                    int(self.at[0]) <= tick - self.half and
                    int(self.at[-1]) >= tick + self.half)
        if len(xt) < self.min_pts:
            # widen to the nearest min_pts anchors (session edge / sparse region)
            centered = False
            i = np.searchsorted(self.at, tick)
            lo = max(0, i - self.min_pts)
            hi = min(len(self.at), i + self.min_pts)
            xt, ys = self.at[lo:hi], self.asec[lo:hi]
        if len(xt) < self.min_pts:
            return None, 0.0, 0.0, len(xt), False, False
        # Do not silently extrapolate a sparse/missing PPS interval indefinitely.
        if min(abs(tick - int(xt[0])), abs(tick - int(xt[-1]))) > self.half and not (
                int(xt[0]) <= tick <= int(xt[-1])):
            return None, 0.0, 0.0, len(xt), False, False
        if min(abs(tick - int(t)) for t in xt) > self.half:
            return None, 0.0, 0.0, len(xt), False, False
        a, b, t0, s0, rms, kept = self._fit(xt, ys)
        n = len(kept)
        # Whole-second PPS need not coincide with the window limits. Allow one
        # PPS spacing there, but flag missing seconds and one-sided retained fits.
        centered = bool(centered and int(kept[0]) < tick < int(kept[-1]) and
                        int(kept[0]) - (tick - self.half) <= 1.01 * self.hz and
                        (tick + self.half) - int(kept[-1]) <= 1.01 * self.hz and
                        np.all(np.diff(kept) <= 1.5 * self.hz))
        if not np.isfinite(a) or not np.isfinite(b) or not np.isfinite(rms) or b <= 0:
            return None, 0.0, 0.0, n, False, False
        rel = a + b * float(tick - t0)              # small-offset domain: no float64 abs-second
        w = int(np.floor(rel))
        return int(s0) + w, float(rel - w), rms, n, True, centered


# ---------------------------------------------------------------------------
# Pulse reconstruction + frame matching
# ---------------------------------------------------------------------------
def rising_edges(edges, active_level):
    """edges: [(seq, level, tick)] -> sorted list of (seq, tick) at the active-going edge."""
    out = []
    for seq, level, tick in sorted(edges, key=lambda e: e[2]):
        if level == active_level:
            out.append((seq, tick))
    return out


def exposures(edges, active_level):
    """Return [(start_tick, end_tick, start_seq, suspect)] pairing active->idle edges."""
    out = []
    start = None
    start_seq = None
    last_seq = None
    pend = False                    # seq gap seen: next emitted exposure suspect
    for seq, level, tick in sorted(edges, key=lambda e: e[2]):
        if last_seq is not None and seq != last_seq + 1:
            pend = True             # lost edge(s) in between; pairing may be wrong
        last_seq = seq
        if level == active_level:
            if start is not None:
                # previous exposure never closed (lost idle edge): emit it as suspect
                out.append((start, None, start_seq, True))
                pend = False
            start = tick
            start_seq = seq
        else:
            if start is not None:
                out.append((start, tick, start_seq, pend))
                start = None
                pend = False
    if start is not None:
        out.append((start, None, start_seq, True))
    return out


def match_frames(exps, trigs, half_period_ticks):
    """Match only preceding triggers within half a period; duplicates/reuse stay unmatched."""
    tt = np.array([t for _, t in trigs], dtype=np.int64) if trigs else np.empty(0, np.int64)
    tseq = [s for s, _ in trigs]
    frames = []
    for (s_tick, e_tick, s_seq, suspect) in exps:
        rec = dict(start_tick=s_tick, end_tick=e_tick, start_seq=s_seq,
                   trig_tick=None, trig_seq=None, suspect=suspect)
        if len(tt):
            i = int(np.searchsorted(tt, s_tick, side='right')) - 1
            duplicate = i > 0 and tt[i - 1] == tt[i]
            if i >= 0 and not duplicate and half_period_ticks > 0 and s_tick - int(tt[i]) <= half_period_ticks:
                rec['trig_tick'] = int(tt[i])
                rec['trig_seq'] = tseq[i]
        frames.append(rec)
    counts = {}
    for rec in frames:
        seq = rec['trig_seq']
        if seq is not None:
            counts[seq] = counts.get(seq, 0) + 1
    for rec in frames:
        if rec['trig_seq'] is not None and counts[rec['trig_seq']] > 1:
            rec['trig_tick'] = rec['trig_seq'] = None
            rec['suspect'] = True
    return frames


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="SensorSync-Logger post-processor")
    ap.add_argument('log', help="log file, or '-' for stdin")
    ap.add_argument('-o', '--out', default='frames.csv', help="per-frame CSV output")
    ap.add_argument('--edges', help="also write a per-edge CSV (all T and S with GNSS time)")
    ap.add_argument('--window', type=float, default=10.0, help="centered fit window, seconds (default 10)")
    ap.add_argument('--convention', choices=['preceding', 'following'], required=True,
                    help="verified ToD<->PPS pairing side; nearest-PPS timing is not sufficient evidence")
    ap.add_argument('--tod-scale', choices=['utc', 'gps'], required=True,
                    help="verified time scale of the receiver's NMEA calendar fields")
    ap.add_argument('--gps', action='store_true', help="output GPS seconds instead of UTC/Unix")
    ap.add_argument('--leap', type=int, help="verified GPS-UTC offset; required for GPS input or output")
    ap.add_argument('--link', default='', help="strobe:trigger channel map, e.g. 0:0,1:1 (default same index)")
    ap.add_argument('--session', type=int, help="process session N (1-based) of a multi-session log")
    args = ap.parse_args()
    if not np.isfinite(args.window) or not 0 < args.window <= 3600:
        ap.error("--window must be finite and in (0, 3600]")
    if (args.gps or args.tod_scale == 'gps') and (args.leap is None or not 0 <= args.leap <= 128):
        ap.error("GPS conversion requires an explicit verified --leap in [0, 128]")

    sessions = parse_log(args.log)
    if args.session is not None and not (1 <= args.session <= len(sessions)):
        ap.error(f"--session {args.session}: log has {len(sessions)} session(s)")
    pick = args.session
    if len(sessions) > 1:
        print(f"!!! {len(sessions)} sessions in this log (reboot resets ticks; sessions never mixed in one fit)",
              file=sys.stderr)
        for i, s in enumerate(sessions, 1):
            ne = sum(len(v) for v in s.trig.values()) + sum(len(v) for v in s.strobe.values())
            tag = f" [{s.session_path}]" if s.session_path else ''
            print(f"!!!   session {i}: lines {s.first_line}-{s.last_line}, {len(s.pps)} PPS, "
                  f"{ne} edges, {len(s.tod_seq)} ToD{tag}", file=sys.stderr)
        if pick is None:
            ap.error("multiple clock epochs: select --session explicitly; other sessions must be processed separately")
        print(f"!!! processing session {pick} only (--session N to override)", file=sys.stderr)
    lg = sessions[(pick or 1) - 1]
    if not np.isfinite(lg.tick_hz) or lg.tick_hz <= 0:
        ap.error("invalid nominal tick frequency in log header")
    if args.tod_scale == 'gps':
        lg.tod = [(seq, tick, sec - args.leap) for seq, tick, sec in lg.tod]

    print(f"parsed: {len(lg.pps)} PPS, {sum(len(v) for v in lg.trig.values())} trig edges, "
          f"{sum(len(v) for v in lg.strobe.values())} strobe edges, {len(lg.tod)} ToD "
          f"({lg.tod_bad} bad), {lg.malformed} malformed, tick_hz={lg.tick_hz:.3f}", file=sys.stderr)
    if lg.malformed:
        print(f"  ! {lg.malformed} malformed data line(s) skipped", file=sys.stderr)
    lost = report_gaps('PPS', [s for s, _ in lg.pps], lg.from_marker)
    for ch, ev in lg.trig.items():
        lost += report_gaps(f'trig[{ch}]', [s for s, _, _ in ev], lg.from_marker)
    for ch, ev in lg.strobe.items():
        lost += report_gaps(f'strobe[{ch}]', [s for s, _, _ in ev], lg.from_marker)
    lost += report_gaps('ToD', lg.tod_seq, lg.from_marker)
    log_complete = bool(lg.from_marker and lg.stopped and lg.final_health and
                        not lg.integrity_issues and not lg.malformed and not lost)
    if not log_complete:
        print(f"  ! log integrity incomplete/unverified: {sorted(lg.integrity_issues)}; "
              "CSV observations will carry flag 0x40", file=sys.stderr)

    anchors, _ = build_anchors(lg, args.convention)
    if len(anchors) < 4:
        print("FATAL: not enough PPS<->ToD anchors to fit; check ToD wiring / --convention", file=sys.stderr)
        return 1
    fitter = CenteredFitter(anchors, lg.tick_hz, args.window)

    def to_out(whole):
        # GPS/leap offsets in INTEGER seconds only
        return whole + args.leap - GPS_EPOCH_UNIX if args.gps else whole

    def split(whole, frac):
        ns = int(round(frac * 1e9))
        if ns >= 1_000_000_000:
            whole += 1; ns -= 1_000_000_000
        return whole, ns

    # channel link map
    link = {}
    if args.link:
        for pair in args.link.split(','):
            try:
                a, b = pair.split(':')
                link[int(a)] = int(b)
            except ValueError:
                ap.error(f"--link: bad pair '{pair}' (expected strobe:trigger ints, e.g. 0:0,1:1)")
    else:
        for sc in lg.strobe_cfg:
            if sc in lg.trig_cfg:
                link[sc] = sc

    # optional per-edge CSV
    if args.edges:
        with open(args.edges, 'x', newline='') as f:
            writer = csv.writer(f)
            writer.writerow("kind,ch,seq,level,tick,sec,ns,fit_rms_ns,n_anchors,centered,fit_valid".split(','))
            for kind, store in (('T', lg.trig), ('S', lg.strobe)):
                for ch, ev in store.items():
                    for seq, level, tick in ev:
                        w, fq, rms, n, ok, cen = fitter.gnss(tick)
                        s, ns = split(to_out(w), fq) if ok else ('', '')
                        writer.writerow([kind, ch, seq, level, tick, s, f"{ns:09d}" if ok else '',
                                         f"{rms*1e9:.0f}" if ok else '', n, int(cen), int(ok)])
        print(f"wrote per-edge CSV: {args.edges}", file=sys.stderr)

    # per-frame CSV
    nframes = 0
    with open(args.out, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(("strobe,name,frame,start_sec,start_ns,end_sec,end_ns,mid_sec,mid_ns,"
                         "dur_us,trig_sec,trig_ns,latency_us,fit_rms_ns,n_anchors,start_seq,trig_seq,flags,"
                         "start_tick,end_tick,trig_tick,association_verified,output_time_scale,"
                         "tod_scale,pps_convention,leap_seconds").split(','))
        for sch, ev in sorted(lg.strobe.items()):
            if not ev:
                continue
            name = lg.strobe_cfg.get(sch, {}).get('name', f'S{sch}')
            active = lg.strobe_cfg.get(sch, {}).get('active_high', 1)
            exps = exposures(ev, active)

            tch = link.get(sch)
            trigs = rising_edges(lg.trig.get(tch, []), lg.trig_cfg.get(tch, {}).get('active_high', 1)) if tch is not None else []
            freq = lg.trig_cfg.get(tch, {}).get('freq', 0.0) if tch is not None else 0.0
            half = int(0.5 * lg.tick_hz / freq) if freq > 0 else 0

            frames = match_frames(exps, trigs, half)
            for k, fr in enumerate(frames):
                sw, sf, rms_s, n_s, ok_s, cen_s = fitter.gnss(fr['start_tick'])
                ew, ef, _, _, ok_e, cen_e = fitter.gnss(fr['end_tick']) if fr['end_tick'] is not None else (
                    None, 0.0, 0.0, 0, False, False)
                flags = 0 if log_complete else 0x40
                if not (ok_s and ok_e):
                    flags |= 0x01
                if fr['end_tick'] is None:
                    flags |= 0x10
                if fr['suspect']:
                    flags |= 0x04
                if not (cen_s and cen_e):
                    flags |= 0x08          # not a centered fit (session edge / gap)
                ss = sns = es = ens = ms = mns = dur_us = ''
                if ok_s:
                    ss, ns = split(to_out(sw), sf)
                    sns = f"{ns:09d}"
                if ok_e:
                    es, ns = split(to_out(ew), ef)
                    ens = f"{ns:09d}"
                if ok_s and ok_e:
                    # Midpoint in a small-offset domain, without an absolute float timestamp.
                    mrel = sf + 0.5 * ((ew - sw) + (ef - sf))
                    mfl = int(np.floor(mrel))
                    ms, ns = split(to_out(sw + mfl), mrel - mfl)
                    mns = f"{ns:09d}"
                    dur_us = f"{((ew - sw) + (ef - sf)) * 1e6:.3f}"
                trig_out = tr_ns = ''
                lat_us = ''
                tseq = fr['trig_seq'] if fr['trig_seq'] is not None else ''
                if fr['trig_tick'] is not None:
                    tw, tf, _, _, ok_t, _ = fitter.gnss(fr['trig_tick'])
                    if ok_t:
                        ts, tns = split(to_out(tw), tf)
                        trig_out, tr_ns = ts, f"{tns:09d}"
                        if ok_s:
                            lat_us = f"{((sw - tw) + (sf - tf))*1e6:.3f}"
                    else:
                        flags |= 0x02      # trigger matched but fit failed
                else:
                    flags |= 0x02          # no trigger matched
                writer.writerow([sch, name, k, ss, sns, es, ens, ms, mns, dur_us,
                                 trig_out, tr_ns, lat_us, f"{rms_s*1e9:.0f}" if ok_s else '', n_s,
                                 fr['start_seq'], tseq, flags, fr['start_tick'], fr['end_tick'],
                                 fr['trig_tick'], 0, 'gps' if args.gps else 'utc', args.tod_scale,
                                 args.convention, args.leap if args.leap is not None else ''])
                nframes += 1

    print(f"wrote {nframes} frames -> {args.out}"
          + ("  (GPS seconds)" if args.gps else "  (UTC/Unix seconds)"), file=sys.stderr)
    print("flags: 0x01 incomplete-time-fit  0x02 no-trigger-time(unmatched/fit-fail)  "
          "0x04 exposure-suspect(lost edge)  0x08 non-centered-fit(edge/gap)  0x10 missing-end-edge  "
          "0x40 log-integrity-incomplete; "
          "trigger matching is heuristic, association_verified=0", file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
