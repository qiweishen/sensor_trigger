import csv
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

    def test_csv_duration_uses_calibrated_time_and_blanks_failed_fit(self):
        hz = 75_007_500  # +100 ppm versus nominal
        lines = ['#SESSION,START,test', '#LOG,SensorSync-logger,1,tick_hz=75000000',
                 '#TRIG,0,FX10E-1,33,50,ah=1', '#STROBE,0,FX10E_EXP-1,18,ah=1']
        for i in range(21):
            body = f'GPZDA,0000{i:02d}.00,09,09,2026,00,00'
            checksum = 0
            for ch in body:
                checksum ^= ord(ch)
            lines += [f'P,{i+1},{(i+1)*hz}', f'Z,{i+1},{(i+1)*hz+hz//5},${body}*{checksum:02X}']
        start = 11 * hz
        lines += [f'T,1,0,1,{start-750}', f'T,2,0,0,{start+3000}',
                  f'S,1,0,1,{start}', f'S,2,0,0,{start+750075}',
                  f'S,3,0,1,{25*hz}', f'S,4,0,0,{27*hz}',
                  '#HFINAL ppsdrops=0 todtrunc=0 tdrops=0 sdrops=0 host=0 skip=0 stall=0',
                  '#SESSION,STOP,test']
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'run.log'; out = Path(folder) / 'frames.csv'
            log.write_text('\n'.join(lines) + '\n')
            subprocess.run([sys.executable, str(ROOT / 'postprocess.py'), str(log), '-o', str(out),
                            '--tod-scale', 'utc', '--convention', 'preceding'], check=True,
                           capture_output=True, text=True)
            with out.open() as f:
                frames = list(csv.DictReader(f))
            self.assertEqual(frames[0]['dur_us'], '10000.000')
            delta_ns = ((int(frames[0]['end_sec']) - int(frames[0]['start_sec'])) * 10**9 +
                        int(frames[0]['end_ns']) - int(frames[0]['start_ns']))
            self.assertEqual(delta_ns, 10_000_000)
            self.assertEqual(frames[1]['dur_us'], '')
            self.assertTrue(int(frames[1]['flags']) & 1)


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
                        if payload.startswith('freq='):
                            payload = payload.split(' ', 1)[1]
                        self.path = payload
                        if self.closed.wait(self.start_delay):
                            return
                        if self.start_error:
                            self.send(self.start_error)
                        self.send(f'#SESSION,START,{self.path}')
                        self.send('#LOG,SensorSync-logger,1,tick_hz=75000000')
                    elif cmd == 'STOP':
                        if self.path is None:
                            if self.idle_reply:
                                self.send('#IDLE,up=123')
                            continue
                        if self.closed.wait(self.stop_delay):
                            return
                        if self.final_health:
                            self.send('#HFINAL ppsdrops=0 todtrunc=0 tdrops=0 sdrops=0 host=0 skip=0 stall=0')
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
        result = self.ctl('start', str(self.path), '0:25,1:25,2:5,3:7')
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
        self.board.start_error = '#ERR,bad_freq,0:50'
        result = self.run_client()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('START rejected: #ERR,bad_freq,0:50', result.stderr)
        self.assertIn('#ERR,bad_freq,0:50', (self.path / 'direct.log').read_text())
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
        self.board.send('#H ppsdrops=0 todtrunc=0 tdrops=0 sdrops=0 host=1 skip=0 stall=0')
        self.wait_failed(log)
        self.assertNotEqual(self.ctl('new', str(self.path)).returncode, 0)
        self.assertEqual(sum(cmd.startswith('START ') for cmd in self.board.commands), 1)


if __name__ == '__main__':
    unittest.main()
