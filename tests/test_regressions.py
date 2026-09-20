import calendar
import csv
import math
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import postprocess as pp


class PostprocessTests(unittest.TestCase):
    def fitter(self, omitted=(), shifted=False):
        hz = 75_000_000
        anchors = np.array([(i * hz + (100000 if shifted and i == 15 else 0),
                             1_780_000_000 + i) for i in range(31) if i not in omitted], dtype=np.int64)
        return pp.CenteredFitter(anchors, hz, 10)

    def test_complete_window_and_boundaries(self):
        fit = self.fitter()
        for sec in (0, 1, 4, 26, 29, 30):
            self.assertTrue(fit.gnss(sec * 75_000_000)[4])
            self.assertFalse(fit.gnss(sec * 75_000_000)[5])
        for sec in (5, 10.25, 15, 25):
            self.assertTrue(fit.gnss(int(sec * 75_000_000))[5])

    def test_anchor_gap_and_rejected_anchor_are_not_centered(self):
        self.assertFalse(self.fitter(omitted=(15,)).gnss(15 * 75_000_000)[5])
        result = self.fitter(shifted=True).gnss(15 * 75_000_000)
        self.assertLess(result[3], 11)
        self.assertTrue(result[4])
        self.assertFalse(result[5])

    def test_missing_time_coverage_is_invalid(self):
        self.assertFalse(self.fitter().gnss(50 * 75_000_000)[4])

    def test_trigger_matching_respects_causality_and_window(self):
        trigs = [(1, 0), (3, 20), (5, 40)]
        for start, expected in [(12, None), (19, None), (20, 3), (21, 3), (30, 3), (-1, None)]:
            frame = pp.match_frames([(start, start + 1, 1, False)], trigs, 10)[0]
            self.assertEqual(frame['trig_seq'], expected)

    def test_duplicate_or_reused_trigger_is_unmatched(self):
        self.assertIsNone(pp.match_frames([(1, 2, 1, False)], [(1, 0), (3, 0)], 10)[0]['trig_seq'])
        frames = pp.match_frames([(1, 2, 1, False), (3, 4, 3, False)], [(1, 0)], 10)
        self.assertTrue(all(fr['trig_seq'] is None and fr['suspect'] for fr in frames))

    HEADER = ['#SESSION,START,test', '#LOG,SensorSync-logger,1,tick_hz=75000000',
              '#TRIG,0,FX10E-1,33,50,ah=1', '#STROBE,0,FX10E_EXP-1,18,ah=1']
    FOOTER = ['#HFINAL ppsdrops=0 todtrunc=0 tdrops=0 sdrops=0 host=0 skip=0 stall=0',
              '#SESSION,STOP,test']

    @staticmethod
    def gnss_lines(hz, seconds=21):
        # PPS i+1 labelled 2026-09-09 00:00:ii UTC, sentence 0.2 s after its PPS
        lines = []
        for i in range(seconds):
            body = f'GPZDA,0000{i:02d}.00,09,09,2026,00,00'
            checksum = 0
            for ch in body:
                checksum ^= ord(ch)
            lines += [f'P,{i + 1},{(i + 1) * hz}', f'Z,{i + 1},{(i + 1) * hz + hz // 5},${body}*{checksum:02X}']
        return lines

    def run_postprocess(self, lines, *options, edges=False):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'run.log';
            out = Path(folder) / 'frames.csv'
            edge_csv = Path(folder) / 'edges.csv'
            log.write_text('\n'.join(lines) + '\n')
            result = subprocess.run([sys.executable, str(ROOT / 'postprocess.py'), str(log), '-o', str(out),
                                     *(('--edges', str(edge_csv)) if edges else ()), *options],
                                    capture_output=True, text=True)
            tables = []
            for path in (out, edge_csv):
                if path.exists():
                    with path.open() as f:
                        tables.append(list(csv.DictReader(f)))
                else:
                    tables.append([])
        return result, tables[0], tables[1]

    def test_csv_duration_uses_calibrated_time_and_blanks_failed_fit(self):
        hz = 75_007_500  # +100 ppm versus nominal
        start = 11 * hz
        lines = self.HEADER + self.gnss_lines(hz) + [
            f'T,1,0,1,{start - 750}', f'T,2,0,0,{start + 3000}',
            f'S,1,0,1,{start}', f'S,2,0,0,{start + 750075}',
            f'S,3,0,1,{25 * hz}', f'S,4,0,0,{27 * hz}'] + self.FOOTER
        result, frames, _ = self.run_postprocess(lines, '--convention', 'preceding')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(frames[0]['dur_us'], '10000.000')
        delta_ns = ((int(frames[0]['end_utc_sec']) - int(frames[0]['start_utc_sec'])) * 10 ** 9 +
                    int(frames[0]['end_ns']) - int(frames[0]['start_ns']))
        self.assertEqual(delta_ns, 10_000_000)
        self.assertEqual(frames[1]['dur_us'], '')
        self.assertTrue(int(frames[1]['flags']) & 1)

    def test_csv_carries_utc_and_gps_seconds(self):
        hz = 75_000_000
        start = 11 * hz  # PPS 11 = 2026-09-09 00:00:10 UTC
        lines = self.HEADER + self.gnss_lines(hz) + [
            f'T,1,0,1,{start - 750}', f'T,2,0,0,{start + 3000}',
            f'S,1,0,1,{start}', f'S,2,0,0,{start + 750000}'] + self.FOOTER
        utc = calendar.timegm((2026, 9, 9, 0, 0, 10))
        for options, leap in (((), 18), (('--leap', '19'), 19)):
            result, frames, edges = self.run_postprocess(lines, '--convention', 'preceding', *options, edges=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            frame = frames[0]
            self.assertEqual((frame['start_utc_sec'], frame['start_ns']), (str(utc), '000000000'))
            for key in ('start', 'end', 'mid', 'trig'):
                self.assertEqual(int(frame[f'{key}_gps_sec']) - int(frame[f'{key}_utc_sec']),
                                 leap - pp.GPS_EPOCH_UNIX)
            self.assertEqual((frame['time_base'], frame['pps_convention'], frame['leap_seconds']),
                             ('gnss', 'preceding', str(leap)))
            self.assertFalse(int(frame['flags']) & 0x20)
            for column in ('output_time_scale', 'tod_scale', 'start_sec'):
                self.assertNotIn(column, frame)
            strobe = next(e for e in edges if e['kind'] == 'S' and e['level'] == '1')
            self.assertEqual((strobe['utc_sec'], strobe['gps_sec'], strobe['ns'], strobe['time_base']),
                             (str(utc), str(utc + leap - pp.GPS_EPOCH_UNIX), '000000000', 'gnss'))

    def test_removed_options_are_rejected(self):
        for options in (('--tod-scale', 'utc'), ('--gps',)):
            result, _, _ = self.run_postprocess(self.HEADER + self.FOOTER, *options)
            self.assertEqual(result.returncode, 2)

    def test_convention_is_required_only_with_pps_and_tod(self):
        result, frames, _ = self.run_postprocess(self.HEADER + self.gnss_lines(75_000_000) + self.FOOTER)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--convention', result.stderr)
        self.assertEqual(frames, [])

    def test_missing_pps_and_tod_free_runs_from_gps_epoch(self):
        hz = 75_000_000
        first = 123 * hz + 4567
        lines = list(self.HEADER)
        for k in range(51):  # 50 Hz: frame 50 lands exactly 1 s after the first trigger
            trig = first + k * (hz // 50)
            lines += [f'T,{2 * k + 1},0,1,{trig}', f'T,{2 * k + 2},0,0,{trig + 3750}',
                      f'S,{2 * k + 1},0,1,{trig + 7500}', f'S,{2 * k + 2},0,0,{trig + 7500 + 750000}']
        result, frames, edges = self.run_postprocess(lines + self.FOOTER, edges=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('FREE-RUN', result.stderr)
        self.assertEqual(len(frames), 51)
        head = frames[0]
        self.assertEqual((head['trig_utc_sec'], head['trig_gps_sec'], head['trig_ns']),
                         (str(pp.GPS_EPOCH_UNIX), '0', '000000000'))
        self.assertEqual((head['start_gps_sec'], head['start_ns']), ('0', '000100000'))
        self.assertEqual((head['end_gps_sec'], head['end_ns']), ('0', '010100000'))
        self.assertEqual((head['mid_gps_sec'], head['mid_ns']), ('0', '005100000'))
        self.assertEqual((head['dur_us'], head['latency_us']), ('10000.000', '100.000'))
        self.assertEqual((head['fit_rms_ns'], head['n_anchors']), ('', '0'))
        tail = frames[50]
        self.assertEqual((tail['trig_utc_sec'], tail['trig_gps_sec'], tail['trig_ns']),
                         (str(pp.GPS_EPOCH_UNIX + 1), '1', '000000000'))
        for frame in frames:
            self.assertEqual(int(frame['flags']), 0x20)
            self.assertEqual((frame['time_base'], frame['pps_convention'], frame['leap_seconds']),
                             ('free_run', '', '0'))
        self.assertEqual((edges[0]['utc_sec'], edges[0]['gps_sec'], edges[0]['ns'], edges[0]['time_base']),
                         (str(pp.GPS_EPOCH_UNIX), '0', '000000000', 'free_run'))

    def test_too_few_anchors_free_runs_instead_of_aborting(self):
        hz = 75_000_000
        lines = self.HEADER + self.gnss_lines(hz, seconds=3) + [
            f'T,1,0,1,{2 * hz}', f'T,2,0,0,{2 * hz + 3750}',
            f'S,1,0,1,{2 * hz + 7500}', f'S,2,0,0,{2 * hz + 757500}'] + self.FOOTER
        result, frames, _ = self.run_postprocess(lines, '--convention', 'preceding')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('check PPS/ToD wiring', result.stderr)
        self.assertEqual((frames[0]['trig_gps_sec'], frames[0]['trig_ns'], frames[0]['time_base']),
                         ('0', '000000000', 'free_run'))

    def test_strobe_polarity_override_recovers_active_low_exposure(self):
        hz = 75_000_000
        lines = list(self.HEADER)  # header declares ah=1
        for k in range(3):  # 50 Hz; line sinks low for 5 ms, 100 us after each trigger
            trig = 5 * hz + k * (hz // 50)
            lines += [f'T,{2 * k + 1},0,1,{trig}', f'T,{2 * k + 2},0,0,{trig + 3750}',
                      f'S,{2 * k + 1},0,0,{trig + 7500}', f'S,{2 * k + 2},0,1,{trig + 7500 + 375000}']
        lines += self.FOOTER
        result, frames, _ = self.run_postprocess(lines, '--strobe-ah', '0:0')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('polarity override', result.stderr)
        self.assertEqual([(f['dur_us'], f['latency_us'], f['strobe_active_high'], f['flags']) for f in frames],
                         [('5000.000', '100.000', '0', '32')] * 3)
        _, frames, _ = self.run_postprocess(lines)  # header polarity: high phase taken as exposure
        self.assertEqual((frames[0]['dur_us'], frames[0]['latency_us'], frames[0]['strobe_active_high']),
                         ('15000.000', '5100.000', '1'))
        for bad in ('0:2', '0', '9:0'):
            self.assertEqual(self.run_postprocess(lines, '--strobe-ah', bad)[0].returncode, 2)

    def test_free_run_clock_origin_and_earlier_edges(self):
        hz = 75_000_000
        clock = pp.FreeRunClock(1000, hz)
        self.assertEqual(clock.gnss(1000), (pp.GPS_EPOCH_UNIX, 0.0, 0.0, 0, True, False))
        whole, frac = clock.gnss(1000 + 90 * 60 * hz + 75)[:2]  # 1.5 h + 1 us
        self.assertEqual(whole, pp.GPS_EPOCH_UNIX + 5400)
        self.assertAlmostEqual(frac, 1e-6, delta=1e-10)
        whole, frac = clock.gnss(1000 - 75)[:2]  # 1 us before the origin
        self.assertEqual(whole, pp.GPS_EPOCH_UNIX - 1)
        self.assertAlmostEqual(frac, 1 - 1e-6, delta=1e-10)

    def test_free_run_origin_is_first_active_trigger_edge(self):
        lg = pp.Log()
        lg.trig_cfg = {0: dict(active_high=1), 2: dict(active_high=0)}
        lg.trig = {0: [(2, 0, 50), (3, 1, 900)], 2: [(1, 1, 40), (2, 0, 700)]}
        lg.strobe = {0: [(1, 1, 10)]}
        self.assertEqual(pp.first_trigger_tick(lg), 700)
        lg.trig = {0: [(2, 0, 50)]}
        self.assertEqual(pp.first_trigger_tick(lg), 10)  # no active-going trigger: earliest edge
        self.assertIsNone(pp.first_trigger_tick(pp.Log()))


class FakeBoard:
    def __init__(self, stop_delay=0, start_delay=0, final_health=True, stop_ack=True,
                 pending_command=b'', fragment_replies=False):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self.stop_delay = stop_delay
        self.start_delay = start_delay
        self.final_health = final_health
        self.stop_ack = stop_ack
        self.pending_command = pending_command
        self.fragment_replies = fragment_replies
        self.sync_reply = True
        self.completion_reply = True
        self.idle_reply = True
        self.status_reply = '#IDLE,up=123'
        self.start_error = None
        self.version = 2
        self.ready_ack = True
        self.ready_delay = 0
        self.rates = {'FX': 50.0, 'JAI': 0.0}
        self.pwm_error = 0
        self.truncated = []
        self.sync_seen = threading.Event()
        self.commands = []
        self.path = None
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, line):
        data = (line + '\r\n').encode()
        chunk = 3 if self.fragment_replies else len(data)
        for offset in range(0, len(data), chunk):
            os.write(self.master, data[offset:offset + chunk])
            if self.fragment_replies and self.closed.wait(.005):
                return

    def run(self):
        buf = self.pending_command  # firmware cmd[] may already contain an unterminated line
        while not self.closed.is_set():
            try:
                if not select.select([self.master], [], [], .05)[0]:
                    continue
                buf += os.read(self.master, 4096)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    cmd = line.decode().strip()
                    self.commands.append(cmd)
                    if len(line.rstrip(b'\r')) > 255:
                        self.send('#ERR,command_too_long')
                        continue
                    if cmd.startswith('START '):
                        payload = cmd[6:]
                        rates = dict(self.rates)
                        if payload.startswith('freq='):
                            spec, _, payload = payload.partition(' ')
                            seen = set()
                            valid = True
                            for item in spec[5:].split(','):
                                group, sep, value = item.partition(':')
                                try:
                                    hz = float(value)
                                except ValueError:
                                    valid = False
                                    break
                                if (not sep or group not in rates or group in seen or not math.isfinite(hz) or
                                        not (hz == 0 or (hz >= 20 if group == 'FX' else 1 <= hz <= 10))):
                                    valid = False
                                    break
                                seen.add(group)
                                rates[group] = hz
                            if not valid:
                                self.send('#ERR,bad_freq')
                                continue
                        if self.start_error:
                            self.send(self.start_error)
                            continue  # v2 rejects the entire START, remaining idle
                        self.path = payload
                        self.rates = rates
                        if self.closed.wait(self.start_delay):
                            return
                        self.send(f'#SESSION,START,{self.path}')
                        self.send(f'#LOG,SensorSync-logger,{self.version},tick_hz=75000000')
                        if self.version == 2 and self.ready_ack:
                            if self.closed.wait(self.ready_delay):
                                return
                            self.send('#READY')
                    elif cmd == 'STOP':
                        if self.path is None:
                            if self.idle_reply:
                                self.send('#IDLE,up=123')
                            continue
                        if self.closed.wait(self.stop_delay):
                            return
                        for record in self.truncated:
                            self.send(record)
                        if self.final_health:
                            extra = (f'pwmerr={self.pwm_error} trunc={len(self.truncated)}'
                                     if self.version == 2 else 'skip=0 stall=0')
                            self.send('#HFINAL ppsdrops=0 todtrunc=0 tdrops=0 sdrops=0 host=0 ' + extra)
                        if self.stop_ack:
                            self.send(f'#SESSION,STOP,{self.path}')
                        self.path = None
                    elif cmd == 's':
                        if self.idle_reply:
                            self.send(self.status_reply)
                    elif cmd:
                        if cmd.startswith('__SENSORSYNC_SYNC_'):
                            self.sync_seen.set()
                            if cmd.endswith('_DONE') and not self.completion_reply:
                                continue
                        if not cmd.startswith('__SENSORSYNC_SYNC_') or self.sync_reply:
                            self.send(f'#ERR,unknown_command,{cmd}')
            except (OSError, ValueError):
                return

    def close(self):
        if self.closed.is_set():
            return
        self.closed.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


class NativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix='sensorsync_tests_')
        cls.client = Path(cls.build.name) / 'session_client'
        cls.channels = Path(cls.build.name) / 'channels_test'
        common = ['g++', '-O2', '-Wall', '-Wextra', '-Werror']
        subprocess.run(common + ['-std=c++20', '-pthread', '-DSESSION_CLIENT_DEMO', str(ROOT / 'session_client.cpp'),
                                 '-o', str(cls.client)], check=True, capture_output=True)
        subprocess.run(common + ['-std=c++17', '-I', str(ROOT / 'tests/fakes'), '-I', str(ROOT),
                                 str(ROOT / 'tests/channels_test.cpp'), str(ROOT / 'channels.cpp'),
                                 '-o', str(cls.channels)], check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix='ss_')
        self.path = Path(self.folder.name)
        self.board = FakeBoard()
        self.env = {**os.environ, 'SENSOR_PORT': self.board.port, 'SESSION_CLIENT': str(self.client),
                    'RECORD_STATE': str(self.path / 'state')}

    def tearDown(self):
        # Best-effort graceful cleanup if an assertion failed during a live run.
        state = self.path / 'state'
        if state.exists():
            subprocess.run(['bash', str(ROOT / 'record_ctl.bash'), 'stop'], env=self.env,
                           capture_output=True, timeout=10)
        self.board.close()
        self.folder.cleanup()

    def ctl(self, *args):
        return subprocess.run(['bash', str(ROOT / 'record_ctl.bash'), *args],
                              env=self.env, capture_output=True, text=True, timeout=12)

    def start(self):
        result = self.ctl('start', str(self.path), 'FX:25,JAI:5')
        self.assertEqual(result.returncode, 0, result.stderr)
        return Path(result.stdout.strip())

    def replace_board(self, **options):
        self.board.close()
        self.board = FakeBoard(**options)
        self.env['SENSOR_PORT'] = self.board.port

    def run_client(self):
        return subprocess.run([str(self.client), self.board.port, str(self.path / 'direct.log'), '1', '50'],
                              capture_output=True, text=True, timeout=8)

    def test_residual_boot_banner_is_recovered_before_start(self):
        self.replace_board(pending_command=b'# SensorSync-logg', fragment_replies=True)
        log = self.start()
        self.assertEqual(self.ctl('stop').returncode, 0)
        diagnostics = Path(str(log) + '.stderr').read_text()
        self.assertIn('#ERR,unknown_command,# SensorSync-logg', diagnostics)
        self.assertIn('__SENSORSYNC_SYNC_', diagnostics)
        data = log.read_text()
        self.assertTrue(data.startswith('#SESSION,START,'))
        self.assertNotIn('#ERR', data)
        self.assertNotIn('__SENSORSYNC_SYNC_', data)
        self.assertEqual(self.board.commands[0], '# SensorSync-logg')
        self.assertEqual(self.board.commands[1], 'STOP')
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)

    def test_overflowed_residual_command_is_recovered(self):
        self.replace_board(pending_command=b'x' * 256)
        log = self.start()
        self.assertEqual(self.ctl('stop').returncode, 0)
        self.assertIn('#ERR,command_too_long', Path(str(log) + '.stderr').read_text())
        self.assertNotIn('#ERR', log.read_text())

    def test_stale_idle_cannot_replace_fresh_barrier(self):
        self.board.sync_reply = False  # STOP and status still return idle
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('startup synchronization timed out', result.stderr)
        self.assertIn('#IDLE,up=', result.stderr)
        self.assertFalse(any(cmd.startswith('START ') for cmd in self.board.commands))
        self.assertFalse((self.path / 'direct.log').exists())

    def test_barrier_without_idle_never_starts_capture(self):
        self.board.idle_reply = False
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('unexpected reply after startup barrier', result.stderr)
        self.assertFalse(any(cmd.startswith('START ') for cmd in self.board.commands))

    def test_idle_without_completion_never_starts_capture(self):
        self.board.completion_reply = False
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('startup synchronization timed out', result.stderr)
        self.assertFalse(any(cmd.startswith('START ') for cmd in self.board.commands))

    def test_non_idle_reply_after_barrier_is_failure(self):
        self.board.status_reply = '#H up=123 en=1'
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('unexpected reply after startup barrier', result.stderr)
        self.assertFalse(any(cmd.startswith('START ') for cmd in self.board.commands))

    def test_real_start_error_is_not_swallowed_as_startup_noise(self):
        self.board.start_error = '#ERR,bad_freq'
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('START rejected (freq=FX:50): #ERR,bad_freq', result.stderr)
        self.assertIn('#ERR,bad_freq', (self.path / 'direct.log').read_text())
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)

    def test_oversized_startup_reply_never_starts_capture(self):
        self.board.status_reply = 'x' * 2049
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('startup reply exceeds 2048-byte line limit', result.stderr)
        self.assertFalse(any(cmd.startswith('START ') for cmd in self.board.commands))

    def test_disconnect_during_startup_never_sends_start(self):
        self.board.sync_reply = False
        with subprocess.Popen([str(self.client), self.board.port, str(self.path / 'direct.log'), '1', '50'],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as proc:
            self.assertTrue(self.board.sync_seen.wait(2))
            self.board.close()
            _, diagnostics = proc.communicate(timeout=5)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn('serial port lost during startup synchronization', diagnostics)
        self.assertFalse(any(cmd.startswith('START ') for cmd in self.board.commands))

    def test_actual_channels_scheduler(self):
        subprocess.run([str(self.channels)], check=True, capture_output=True)

    def test_command_uses_named_groups_and_waits_for_ready(self):
        self.board.ready_delay = .5
        begin = time.monotonic()
        log = self.start()
        self.assertGreaterEqual(time.monotonic() - begin, .5)
        starts = [cmd for cmd in self.board.commands if cmd.startswith('START ')]
        self.assertEqual(starts, [f'START freq=FX:25,JAI:5 {log}'])
        self.assertEqual(self.ctl('stop').returncode, 0)
        self.assertIn('#READY', log.read_text())

    def test_session_marker_without_ready_does_not_start(self):
        self.board.ready_ack = False
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('#READY acknowledgement missing', result.stderr)

    def test_v1_firmware_cannot_silently_acknowledge_v2_start(self):
        self.board.version = 1
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('SensorSync protocol v2 required', result.stderr)

    def test_v2_final_health_and_audited_high_pulse_cut(self):
        log = self.start()
        self.board.truncated = ['#TRUNC,0,1,75000000']
        self.assertEqual(self.ctl('stop').returncode, 0)
        self.assertIn('pwmerr=0 trunc=1', log.read_text())

    def test_v2_pwm_error_is_fatal(self):
        log = self.start()
        self.board.send('#H ppsdrops=0 todtrunc=0 tdrops=0 sdrops=0 host=0 pwmerr=1 trunc=0')
        self.wait_failed(log)
        self.assertNotEqual(self.ctl('stop').returncode, 0)
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)

    def test_start_waits_for_ack_stop_waits_for_final_tail(self):
        self.board.start_delay = .4
        self.board.stop_delay = .7
        t0 = time.monotonic()
        log = self.start()
        self.assertGreaterEqual(time.monotonic() - t0, .4)
        self.assertIn('recording ->', self.ctl('status').stdout)
        t0 = time.monotonic()
        stopped = self.ctl('stop')
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertGreaterEqual(time.monotonic() - t0, .7)
        data = log.read_text()
        self.assertIn('#HFINAL', data)
        self.assertIn('#SESSION,STOP,', data)
        self.assertEqual(Path(str(log) + '.status').read_text().strip(), 'DONE')
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)
        self.assertEqual(self.ctl('status').stdout.strip(), 'idle')

    def test_missing_stop_ack_is_failure(self):
        self.board.stop_ack = False
        self.start()
        result = self.ctl('stop')
        self.assertNotEqual(result.returncode, 0)

    def test_missing_final_health_is_failure(self):
        self.board.final_health = False
        self.start()
        self.assertNotEqual(self.ctl('stop').returncode, 0)

    def test_unexpected_idle_fails_without_restart(self):
        log = self.start()
        self.board.send('#IDLE,up=5000')
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            status = Path(str(log) + '.status').read_text().strip()
            if status == 'FAILED':
                break
            time.sleep(.05)
        self.assertEqual(status, 'FAILED')
        self.assertNotEqual(self.ctl('stop').returncode, 0)
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)

    def test_bad_port_does_not_report_started(self):
        self.env['SENSOR_PORT'] = str(self.path / 'no-port')
        result = self.ctl('start', str(self.path))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')

    def wait_failed(self, log):
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            if Path(str(log) + '.status').read_text().strip() == 'FAILED':
                return
            time.sleep(.05)
        self.fail('recorder did not report failure')

    def test_serial_disconnect_fails_without_reopening(self):
        log = self.start()
        self.board.close()
        self.wait_failed(log)
        self.assertIn('FAILED', self.ctl('status').stdout)
        self.assertNotEqual(self.ctl('stop').returncode, 0)
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)

    def test_command_error_during_capture_still_fails_without_restart(self):
        log = self.start()
        self.board.send('#ERR,unknown_command,unexpected')
        self.wait_failed(log)
        self.assertNotEqual(self.ctl('stop').returncode, 0)
        self.assertIn('#ERR,unknown_command,unexpected', log.read_text())
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)

    def test_reported_loss_fails_and_new_does_not_hide_failure(self):
        log = self.start()
        self.board.send('#H ppsdrops=0 todtrunc=0 tdrops=0 sdrops=0 host=1 pwmerr=0 trunc=0')
        self.wait_failed(log)
        self.assertNotEqual(self.ctl('new', str(self.path)).returncode, 0)
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)


if __name__ == '__main__':
    unittest.main()
