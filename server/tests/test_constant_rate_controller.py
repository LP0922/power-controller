import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from powder_sampling_control.dispensing_algorithm.calibration import (
    ConstantRateController,
)


class ConstantRateControllerTests(unittest.TestCase):
    def test_accepts_absolute_window_coordinate_from_web_client(self):
        controller = ConstantRateController(
            target_rate_mg_s=10.0,
            frequency_hz=40,
            initial_duty_permyriad=2000,
            # Web position 300 is translated to the LA10 coordinate 1000.
            initial_window_units=1000,
        )

        self.assertEqual(controller.window_position_units, 1000)


if __name__ == "__main__":
    unittest.main()
