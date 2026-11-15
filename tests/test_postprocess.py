import calendar
import csv
import sys
import tempfile
import time
import unittest
from decimal import Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import postprocess as pp

HZ = 75_000_000
T0 = calendar.timegm((2026, 11, 9, 3, 0, 0))
HEADER = ['#SESSION,START,test', '#LOG,SensorSync-logger,3,tick_hz=75000000.000',
          '#TRIG,0,FX10E-1,33,25.00000000', '#STROBE,0,FX10E_EXP-1,18,ah=1']


def nmea(body):
    x = 0
    for c in body:
        x ^= ord(c)
    return f'${body}*{x:02X}'


def zda(sec):
    y, mo, d, h, mi, s = time.gmtime(sec)[:6]
    return nmea(f'GPZDA,{h:02d}{mi:02d}{s:02d}.00,{d:02d},{mo:02d},{y},00,00')


def gnss(hz, seconds=30, mislabel=None):
    """PPS i at tick i*hz; its ZDA 0.2 s later labels T0 + i."""
    lines = []
    for i in range(seconds):
        label = T0 + i + (1 if i == mislabel else 0)
        lines += [f'P,{i + 1},{round(i * hz)}', f'Z,{i + 1},{round((i + 0.2) * hz)},{zda(label)}']
    return lines


def frame(hz, at, delay=100e-6, exposure=5e-3, active_high=True):
    """Trigger at `at` s, exposure from at + delay for `exposure` s (true seconds)."""
    on, off = int(active_high), int(not active_high)
    return [f'T,1,0,1,{round(at * hz)}', f'S,1,0,{on},{round((at + delay) * hz)}',
            f'S,2,0,{off},{round((at + delay + exposure) * hz)}']


def run(lines, *options):
    with tempfile.TemporaryDirectory() as folder:
        log, out = Path(folder) / 'run.log', Path(folder) / 'run.csv'
        log.write_text('\n'.join(lines) + '\n')
        pp.main([str(log), '-o', str(out), *options])
        with out.open() as fh:
            return list(csv.DictReader(fh))


class PostprocessTests(unittest.TestCase):
    def test_nmea_time(self):
        self.assertEqual(pp.nmea_time(zda(T0)), T0)
        self.assertEqual(pp.nmea_time(nmea('GNRMC,030000.00,A,0,N,0,E,0,0,091126,,,A')), T0)
        self.assertIsNone(pp.nmea_time(zda(T0)[:-1] + '0'))	 # bad checksum
        self.assertIsNone(pp.nmea_time(nmea('GPZDA,030000.50,09,11,2026,00,00')))  # not on the second
        self.assertIsNone(pp.nmea_time(nmea('GNRMC,030000.00,V,0,N,0,E,0,0,091126,,,N')))  # no fix

    def test_fit_corrects_clock_rate(self):
        hz = HZ * (1 + 100e-6)  # board clock 100 ppm fast
        row, = run(HEADER + gnss(hz) + frame(hz, 10.5))
        half_tick = 0.5 / hz  # synthetic edges are rounded to whole ticks
        for key, seconds in (('trigger_utc', 10.5), ('start_utc', 10.5001), ('mid_utc', 10.5026)):
            self.assertAlmostEqual(float(Decimal(row[key]) - T0), seconds, delta=half_tick)
        self.assertAlmostEqual(float(row['dur_us']), 5000, delta=2e6 * half_tick)
        self.assertAlmostEqual(float(row['latency_us']), 100, delta=2e6 * half_tick)
        self.assertEqual(row['trigger_seq'], '1')

    def test_mislabeled_second_is_ignored(self):
        clean = run(HEADER + gnss(HZ) + frame(HZ, 12.25))
        self.assertEqual(run(HEADER + gnss(HZ, mislabel=12) + frame(HZ, 12.25)), clean)

    def test_gps_seconds(self):
        row, = run(HEADER + gnss(HZ) + frame(HZ, 10.5), '--gps')
        self.assertEqual(row['start_gps'], f'{T0 + 18 - pp.GPS_EPOCH + 10}.500100000')

    def test_active_low_exposure(self):
        lines = HEADER + gnss(HZ) + frame(HZ, 10.5, active_high=False)
        row, = run(lines, '--strobe-ah', '0:0')
        self.assertEqual((row['dur_us'], row['latency_us']), ('5000.000', '100.000'))
        low = [line.replace('ah=1', 'ah=0') for line in lines]  # same from the header
        self.assertEqual(run(low), [row])

    def test_free_run_without_gnss(self):
        row, = run(HEADER + frame(HZ, 2.0))
        self.assertEqual((row['trigger_rel'], row['start_rel']), ('0.000000000', '0.000100000'))
        self.assertEqual(row['dur_us'], '5000.000')

    def test_trigger_older_than_one_period_is_unmatched(self):
        lines = HEADER + gnss(HZ) + frame(HZ, 10.5, delay=0.05)  # 25 Hz: period 40 ms
        row, = run(lines)
        self.assertEqual((row['trigger_utc'], row['latency_us']), ('', ''))

    def test_old_logs_with_falling_trigger_edges(self):
        lines = HEADER + gnss(HZ) + [f'T,1,0,1,{round(10.5 * HZ)}', f'T,2,0,0,{round(10.50005 * HZ)}',
                                     f'S,1,0,1,{round(10.5001 * HZ)}', f'S,2,0,0,{round(10.5051 * HZ)}']
        row, = run(lines)
        self.assertEqual((row['latency_us'], row['trigger_seq']), ('100.000', '1'))


if __name__ == '__main__':
    unittest.main()
