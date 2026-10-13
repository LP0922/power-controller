import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from powder_sampling_control.dispensing_algorithm.calibration import (
    RateSearchResult,
    _is_stable,
    _not_converged_band,
    _steer_next,
    apply_rate_search,
)


class IsStableTests(unittest.TestCase):
    def test_in_band_low_cv_is_stable(self):
        self.assertTrue(_is_stable({
            "steady_rate_mg_s": 18.0, "cv_pct": 12.0, "interruption_fraction": 0.05,
        }))

    def test_below_band_is_not_stable(self):
        self.assertFalse(_is_stable({
            "steady_rate_mg_s": 8.0, "cv_pct": 12.0, "interruption_fraction": 0.05,
        }))

    def test_above_band_is_not_stable(self):
        self.assertFalse(_is_stable({
            "steady_rate_mg_s": 35.0, "cv_pct": 12.0, "interruption_fraction": 0.05,
        }))

    def test_high_cv_is_not_stable(self):
        self.assertFalse(_is_stable({
            "steady_rate_mg_s": 18.0, "cv_pct": 45.0, "interruption_fraction": 0.05,
        }))

    def test_high_interruption_is_not_stable(self):
        self.assertFalse(_is_stable({
            "steady_rate_mg_s": 18.0, "cv_pct": 12.0, "interruption_fraction": 0.5,
        }))

    def test_none_metrics_are_tolerated(self):
        self.assertTrue(_is_stable({
            "steady_rate_mg_s": 18.0, "cv_pct": None, "interruption_fraction": None,
        }))


class SteerNextTests(unittest.TestCase):
    def test_interrupted_raises_frequency(self):
        freq, window, duty, note = _steer_next(
            rate=18.0, interruption_fraction=0.35, freq=40, window=250, duty=2000,
        )
        self.assertEqual((freq, window, duty), (50, 250, 2000))
        self.assertIn("升频", note)

    def test_interrupted_at_max_freq_raises_duty(self):
        freq, window, duty, note = _steer_next(
            rate=18.0, interruption_fraction=0.35, freq=80, window=250, duty=2000,
        )
        self.assertEqual((freq, window, duty), (80, 250, 2500))

    def test_fast_closes_window(self):
        freq, window, duty, note = _steer_next(
            rate=45.0, interruption_fraction=0.05, freq=40, window=250, duty=2000,
        )
        self.assertEqual((freq, window, duty), (40, 200, 2000))
        self.assertIn("关窗", note)

    def test_fast_at_min_window_drops_duty(self):
        freq, window, duty, note = _steer_next(
            rate=45.0, interruption_fraction=0.05, freq=40, window=100, duty=2000,
        )
        self.assertEqual((freq, window, duty), (40, 100, 1500))

    def test_fast_at_floor_drops_frequency(self):
        freq, window, duty, note = _steer_next(
            rate=45.0, interruption_fraction=0.05, freq=40, window=100, duty=1000,
        )
        self.assertEqual((freq, window, duty), (30, 100, 1000))

    def test_slow_opens_window(self):
        freq, window, duty, note = _steer_next(
            rate=5.0, interruption_fraction=0.05, freq=40, window=250, duty=2000,
        )
        self.assertEqual((freq, window, duty), (40, 300, 2000))
        self.assertIn("开窗", note)

    def test_in_band_returns_none(self):
        self.assertIsNone(_steer_next(
            rate=18.0, interruption_fraction=0.05, freq=40, window=250, duty=2000,
        ))

    def test_none_interruption_fraction_ignored(self):
        self.assertIsNone(_steer_next(
            rate=18.0, interruption_fraction=None, freq=40, window=250, duty=2000,
        ))


class NotConvergedBandTests(unittest.TestCase):
    def test_builds_not_found_band(self):
        band = _not_converged_band(freq=40, window=250)
        self.assertFalse(band["found"])
        self.assertEqual(band["stable_point_count"], 0)
        self.assertIsNone(band["recommended"])
        self.assertIn("40Hz", band["notes"])


class ApplyRateSearchTests(unittest.TestCase):
    def _result(self, point):
        result = RateSearchResult(
            powder_id="powder-test-123",
            powder_name="测试粉",
            target_rate_range_mg_s=[10.0, 30.0],
        )
        result.locked_point = point
        result.stable_band = {"found": True, "recommended": point}
        return result

    def test_maps_locked_point_to_initial(self):
        point = {
            "frequency_hz": 25, "duty_permyriad": 2000, "window_units": 250,
            "mean_rate_mg_s": 18.0, "cv_pct": 10.0,
        }
        session = apply_rate_search(self._result(point))
        self.assertEqual(session["initial"]["frequency_hz"], 25)
        self.assertEqual(session["initial"]["duty_permyriad"], 2000)
        self.assertEqual(session["initial"]["window_position_units"], 250)

    def test_initial_uses_dispense_key(self):
        # start_dispense reads "window_position_units", not "window_units".
        point = {
            "frequency_hz": 25, "duty_permyriad": 2000, "window_units": 300,
            "mean_rate_mg_s": 18.0, "cv_pct": 10.0,
        }
        session = apply_rate_search(self._result(point))
        self.assertIn("window_position_units", session["initial"])
        self.assertEqual(session["initial"]["window_position_units"], 300)

    def test_coarse_rate_clamped_to_band(self):
        point = {
            "frequency_hz": 25, "duty_permyriad": 2000, "window_units": 250,
            "mean_rate_mg_s": 45.0, "cv_pct": 10.0,
        }
        session = apply_rate_search(self._result(point))
        overrides = session["profile_overrides"]
        for target in (100, 300, 500):
            self.assertLessEqual(overrides[target]["coarse_rate_mg_s"], 30.0)

    def test_empty_result_returns_empty_dict(self):
        result = RateSearchResult(target_rate_range_mg_s=[10.0, 30.0])
        self.assertEqual(apply_rate_search(result), {})

    def test_max_flow_rate_is_double_band_hi(self):
        point = {
            "frequency_hz": 25, "duty_permyriad": 2000, "window_units": 250,
            "mean_rate_mg_s": 18.0, "cv_pct": 10.0,
        }
        session = apply_rate_search(self._result(point))
        self.assertEqual(session["maximum_flow_rate_mg_s"], 60.0)


class RateSearchResultTests(unittest.TestCase):
    def test_to_dict_round_trip_shape(self):
        result = RateSearchResult(
            powder_id="p1", powder_name="粉", created_at="t",
            target_rate_range_mg_s=[10.0, 30.0],
        )
        result.locked_point = {"frequency_hz": 25, "mean_rate_mg_s": 18.0}
        d = result.to_dict()
        self.assertEqual(d["powder_id"], "p1")
        self.assertEqual(d["target_rate_range_mg_s"], [10.0, 30.0])
        self.assertEqual(d["locked_point"]["frequency_hz"], 25)


import threading
import time

import powder_sampling_control.dispensing_algorithm.calibration as rbs


class _Measurement:
    def __init__(self, mass):
        self.mass_mg = mass
        self.raw_count = mass


class _VibSettings:
    def __init__(self):
        self.enabled = False


class _Status:
    def __init__(self):
        self.position_units = 250
        self.is_running = False


class _FakeAT:
    """Balance that accrues mass at a frequency-dependent rate."""

    def __init__(self, rate_fn):
        self.rate_fn = rate_fn
        self.freq = 40
        self.window = 250
        self._t0 = None
        self._mass = 0.0

    def read_measurement(self):
        if self._t0 is None:
            self._t0 = time.monotonic()
        self._mass = (time.monotonic() - self._t0) * self.rate_fn(self.freq, self.window)
        return _Measurement(self._mass)

    def zero(self, authorized=True):
        self._t0 = time.monotonic()
        self._mass = 0.0


class _FakeLA10:
    def __init__(self, at):
        self._at = at
        self._status = _Status()
        self._vib = _VibSettings()

    def read_status(self):
        return self._status

    def enable(self):
        pass

    def move_relative(self, extend, distance_mm, speed_mm_s):
        # _move_window_to_position passes distance_mm = delta / _POSITION_UNITS_PER_MM,
        # so the units delta is distance_mm * _POSITION_UNITS_PER_MM.
        delta = round(distance_mm * rbs._POSITION_UNITS_PER_MM)
        if extend:
            self._status.position_units += delta
        else:
            self._status.position_units -= delta
        self._at.window = self._status.position_units

    def start_vibration(self, frequency_hz, duty_permyriad):
        self._at.freq = frequency_hz
        self._vib.enabled = True

    def read_vibration_settings(self):
        return self._vib

    def stop_vibration(self):
        self._vib.enabled = False

    def wait_until_stopped(self, timeout_s, poll_interval_s=0.1, require_running_transition=False):
        # move_relative applies synchronously in the fake, so the window is
        # already settled at the target by the time we wait.
        return self._status


class _FakeReader:
    def __init__(self, rate_fn):
        self._lock = threading.Lock()
        self._at = _FakeAT(rate_fn)
        self._la10 = _FakeLA10(self._at)


class RateSearchEndToEndTests(unittest.TestCase):
    def setUp(self):
        self._saved = {
            k: getattr(rbs, k)
            for k in ("STARTUP_STEP_DURATION_S", "APPROACH_STEP_DURATION_S",
                      "VERIFY_STEP_DURATION_S")
        }
        rbs.STARTUP_STEP_DURATION_S = 1.5
        rbs.APPROACH_STEP_DURATION_S = 1.5
        rbs.VERIFY_STEP_DURATION_S = 1.5

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(rbs, k, v)

    def _run(self, rate_fn):
        reader = _FakeReader(rate_fn)
        cancel = threading.Event()
        progress = lambda phase, pct: None  # noqa: E731
        return rbs.execute_rate_search(reader, cancel, progress)

    def test_low_flow_locks_in_band(self):
        # 40 Hz → 14 mg/s, already inside 10–30 mg/s, continuous flow.
        result = self._run(lambda freq, window: 0.35 * freq)
        self.assertIsNotNone(result.locked_point)
        self.assertTrue(result.stable_band["found"])
        self.assertGreaterEqual(result.locked_point["mean_rate_mg_s"], 10.0)
        self.assertLessEqual(result.locked_point["mean_rate_mg_s"], 30.0)

    def test_high_flow_closes_window_into_band(self):
        # 40 Hz / 250窗 → 120 mg/s; the search must close the window (not the
        # frequency) to pull the rate back into 10–30 mg/s.
        def rate_fn(freq, window):
            return 3.0 * freq if window >= 250 else 0.5 * freq

        result = self._run(rate_fn)
        self.assertIsNotNone(result.locked_point)
        self.assertTrue(result.stable_band["found"])
        self.assertEqual(result.locked_point["frequency_hz"], 40)
        self.assertLess(result.locked_point["window_units"], 250)
        self.assertGreaterEqual(result.locked_point["mean_rate_mg_s"], 10.0)
        self.assertLessEqual(result.locked_point["mean_rate_mg_s"], 30.0)


if __name__ == "__main__":
    unittest.main()
