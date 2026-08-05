#!/usr/bin/env python3
"""
postprocess.py - turn a SensorSync-Logger raw log into per-frame GNSS times.

The board logs only raw ticks + raw NMEA (it computes no GNSS time). This script:
  1. parses the log (#LOG/#TRIG/#STROBE header + P/T/S/Z lines),
  2. parses each NMEA sentence and pairs it with a PPS edge (auto-detecting the
     sentence<->PPS convention, or forced with --convention),
  3. builds (pps_tick, gnss_second) anchors and, for every trigger/strobe edge,
     does a CENTERED sliding-window robust linear fit tick->GNSS time -- which
     removes the causal extrapolation bias of an on-board trailing fit,
  4. reconstructs trigger pulses and exposure windows, matches frames, and writes
     a per-frame CSV (and optionally a per-edge CSV).

Only dependency: numpy.

Usage:
  python3 postprocess.py LOG.txt -o frames.csv
  python3 postprocess.py LOG.txt --gps --window 10 --convention auto
  python3 postprocess.py LOG.txt --edges edges.csv --link 0:0,1:1
"""

import argparse
import sys
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
    if star < 0 or star + 3 > len(s):
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
                    if line.startswith('#LOG'):
                        for tok in line.split(','):
                            if tok.startswith('tick_hz='):
                                cur.tick_hz = float(tok[8:])
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
    a = np.asarray(sorted(seqs))
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
def build_anchors(lg, convention='auto', tol_ppm=200.0, verbose=True):
    """Return sorted anchors ndarray [[pps_tick, unix_sec], ...] and the median lag (s)."""
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
        elif convention == 'following':
            cand = after
        else:  # auto: nearest edge (correct when receiver latency < 0.5 s, the usual case)
            if before is None:
                cand = after
            elif after is None:
                cand = before
            else:
                cand = before if (z_tick - before) <= (after - z_tick) else after
        if cand is None:
            continue
        lag = (z_tick - cand) / hz          # + => sentence after its PPS (preceding convention)
        if abs(lag) > 1.5:                  # no PPS anywhere near this sentence
            continue
        lags.append(lag)
        anchors[int(cand)] = int(sec)       # last write wins; duplicates agree if consistent
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
        if convention == 'auto' and abs(med_lag) > 0.5:
            print("  ! median lag > 0.5 s: receiver latency is high; if times look off by 1 s, "
                  "force --convention preceding/following", file=sys.stderr)
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
        b, a = np.polyfit(x, y, 1)
        r = y - (a + b * x)
        rms = float(np.sqrt(np.mean(r * r))) if len(r) else 0.0
        return a, b, t0, s0, rms, len(x)

    def gnss(self, tick):
        """Return (whole_sec_int, frac_float, rms_residual_s, n_anchors, ok, centered)."""
        lo = np.searchsorted(self.at, tick - self.half, 'left')
        hi = np.searchsorted(self.at, tick + self.half, 'right')
        xt, ys = self.at[lo:hi], self.asec[lo:hi]
        centered = lo > 0 and hi < len(self.at)     # anchors on both sides -> true centered fit
        if len(xt) < self.min_pts:
            # widen to the nearest min_pts anchors (session edge / sparse region)
            centered = False
            i = np.searchsorted(self.at, tick)
            lo = max(0, i - self.min_pts)
            hi = min(len(self.at), i + self.min_pts)
            xt, ys = self.at[lo:hi], self.asec[lo:hi]
        if len(xt) < 2:
            return None, 0.0, 0.0, len(xt), False, False
        a, b, t0, s0, rms, n = self._fit(xt, ys)
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
                out.append((start, tick, start_seq, True))
                pend = False
            start = tick
            start_seq = seq
        else:
            if start is not None:
                out.append((start, tick, start_seq, pend))
                start = None
                pend = False
    return out


def match_frames(exps, trigs, half_period_ticks):
    """For each exposure, nearest trigger rise within half a period. Returns list of dict."""
    tt = np.array([t for _, t in trigs], dtype=np.int64) if trigs else np.empty(0, np.int64)
    tseq = [s for s, _ in trigs]
    frames = []
    for (s_tick, e_tick, s_seq, suspect) in exps:
        rec = dict(start_tick=s_tick, end_tick=e_tick, start_seq=s_seq,
                   trig_tick=None, trig_seq=None, suspect=suspect)
        if len(tt):
            i = np.searchsorted(tt, s_tick)
            best, bestd = None, None
            for j in (i - 1, i):
                if 0 <= j < len(tt):
                    d = abs(int(tt[j]) - s_tick)
                    if bestd is None or d < bestd:
                        bestd, best = d, j
            if best is not None and (half_period_ticks == 0 or bestd <= half_period_ticks):
                rec['trig_tick'] = int(tt[best])
                rec['trig_seq'] = tseq[best]
        frames.append(rec)
    return frames


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="SensorSync-Logger post-processor")
    ap.add_argument('log', help="log file, or '-' for stdin")
    ap.add_argument('-o', '--out', default='frames.csv', help="per-frame CSV output")
    ap.add_argument('--edges', help="also write a per-edge CSV (all T and S with GNSS time)")
    ap.add_argument('--window', type=float, default=10.0, help="centered fit window, seconds (default 10)")
    ap.add_argument('--convention', choices=['auto', 'preceding', 'following'], default='auto',
                    help="ToD<->PPS pairing side (default auto = nearest PPS)")
    ap.add_argument('--gps', action='store_true', help="output GPS seconds instead of UTC/Unix")
    ap.add_argument('--leap', type=int, default=18, help="GPS-UTC offset for --gps (default 18)")
    ap.add_argument('--link', default='', help="strobe:trigger channel map, e.g. 0:0,1:1 (default same index)")
    ap.add_argument('--session', type=int, help="process session N (1-based) of a multi-session log")
    args = ap.parse_args()

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
            pick = max(range(len(sessions)), key=lambda i: len(sessions[i].pps)) + 1
        print(f"!!! processing session {pick} only (--session N to override)", file=sys.stderr)
    lg = sessions[(pick or 1) - 1]

    print(f"parsed: {len(lg.pps)} PPS, {sum(len(v) for v in lg.trig.values())} trig edges, "
          f"{sum(len(v) for v in lg.strobe.values())} strobe edges, {len(lg.tod)} ToD "
          f"({lg.tod_bad} bad), {lg.malformed} malformed, tick_hz={lg.tick_hz:.3f}", file=sys.stderr)
    if lg.malformed:
        print(f"  ! {lg.malformed} malformed data line(s) skipped", file=sys.stderr)
    report_gaps('PPS', [s for s, _ in lg.pps], lg.from_marker)
    for ch, ev in lg.trig.items():
        report_gaps(f'trig[{ch}]', [s for s, _, _ in ev], lg.from_marker)
    for ch, ev in lg.strobe.items():
        report_gaps(f'strobe[{ch}]', [s for s, _, _ in ev], lg.from_marker)
    report_gaps('ToD', lg.tod_seq, lg.from_marker)

    anchors, _ = build_anchors(lg, args.convention)
    if len(anchors) < 2:
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
        with open(args.edges, 'w') as f:
            f.write("kind,ch,seq,level,tick,sec,ns,fit_rms_ns,n_anchors,centered\n")
            for kind, store in (('T', lg.trig), ('S', lg.strobe)):
                for ch, ev in store.items():
                    for seq, level, tick in ev:
                        w, fq, rms, n, ok, cen = fitter.gnss(tick)
                        if not ok:
                            continue
                        s, ns = split(to_out(w), fq)
                        f.write(f"{kind},{ch},{seq},{level},{tick},{s},{ns:09d},"
                                f"{rms*1e9:.0f},{n},{int(cen)}\n")
        print(f"wrote per-edge CSV: {args.edges}", file=sys.stderr)

    # per-frame CSV
    nframes = 0
    with open(args.out, 'w') as f:
        f.write("strobe,name,frame,start_sec,start_ns,end_sec,end_ns,mid_sec,mid_ns,"
                "dur_us,trig_sec,trig_ns,latency_us,fit_rms_ns,n_anchors,start_seq,trig_seq,flags\n")
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
                ew, ef, _, _, ok_e, _ = fitter.gnss(fr['end_tick'])
                if not (ok_s and ok_e):
                    continue
                flags = 0
                if fr['suspect']:
                    flags |= 0x04
                if not cen_s:
                    flags |= 0x08          # not a centered fit (session edge / gap)
                # mid in small-offset domain (no absolute float64 second)
                mrel = sf + 0.5 * ((ew - sw) + (ef - sf))
                mfl = int(np.floor(mrel))
                mw, mf = sw + mfl, mrel - mfl
                dur_us = (fr['end_tick'] - fr['start_tick']) * 1e6 / lg.tick_hz   # tick-derived
                trig_out = tr_ns = ''
                lat_us = ''
                tseq = fr['trig_seq'] if fr['trig_seq'] is not None else ''
                if fr['trig_tick'] is not None:
                    tw, tf, _, _, ok_t, _ = fitter.gnss(fr['trig_tick'])
                    if ok_t:
                        ts, tns = split(to_out(tw), tf)
                        trig_out, tr_ns = ts, f"{tns:09d}"
                        lat_us = f"{((sw - tw) + (sf - tf))*1e6:.3f}"
                    else:
                        flags |= 0x02      # trigger matched but fit failed
                else:
                    flags |= 0x02          # no trigger matched
                ss, sns = split(to_out(sw), sf)
                es, ens = split(to_out(ew), ef)
                ms, mns = split(to_out(mw), mf)
                f.write(f"{sch},{name},{k},{ss},{sns:09d},{es},{ens:09d},{ms},{mns:09d},"
                        f"{dur_us:.3f},{trig_out},{tr_ns},{lat_us},{rms_s*1e9:.0f},{n_s},"
                        f"{fr['start_seq']},{tseq},{flags}\n")
                nframes += 1

    print(f"wrote {nframes} frames -> {args.out}"
          + ("  (GPS seconds)" if args.gps else "  (UTC/Unix seconds)"), file=sys.stderr)
    print("flags: 0x02 no-trigger-time(unmatched/fit-fail)  0x04 exposure-suspect(lost edge)  "
          "0x08 non-centered-fit(edge/gap)", file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
