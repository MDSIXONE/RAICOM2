import sys
import unittest
from pathlib import Path

import cv2 as cv
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from line_follow import LineProcessor, validate_line_config


class LineFollowTest(unittest.TestCase):
    def setUp(self):
        self.config = {
            "lower": [0, 0, 0],
            "upper": [180, 255, 30],
            "crop_top": 150,
            "crop": [0, 319],
            "line_width": [5, 100],
            "pid_p": 396,
            "pid_i": 0,
            "pid_d": 30,
            "straight_speed": 8,
            "turn_move_speed": 6,
            "turn_speed_min": 16,
            "turn_speed_max": 16,
            "direction": 1,
            "control_hz": 12,
            "lost_confirm_frames": 5,
            "probe_steps": 6,
            "probe_step_duration": 1.167,
            "probe_pause": 0.5,
            "probe_turn_speed": -16,
            "probe_cooldown": 5,
            "surge_line_width": 75,
            "surge_frames": 3,
            "surge_turn_duration": 4.2,
            "surge_turn_speed": -16,
        }

    def test_detects_pure_black_line(self):
        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        cv.rectangle(frame, (145, 170), (175, 239), (0, 0, 0), -1)
        result = LineProcessor(self.config).process(frame)
        self.assertTrue(result.found)
        self.assertAlmostEqual(result.center_x, 160.0, delta=2.0)

    def test_lost_frame_has_no_motion_detection(self):
        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        result = LineProcessor(self.config).process(frame)
        self.assertFalse(result.found)
        self.assertIsNone(result.center_x)
        self.assertEqual(result.pid, 0.0)

    def test_applies_legacy_pid_scale(self):
        processor = LineProcessor(self.config)
        self.assertAlmostEqual(processor.pid.p, 0.396)
        self.assertAlmostEqual(processor.pid.i, 0.0)
        self.assertAlmostEqual(processor.pid.d, 0.03)

        updated = dict(self.config, pid_p=271, pid_i=2, pid_d=12)
        processor.update_config(updated)
        self.assertAlmostEqual(processor.pid.p, 0.271)
        self.assertAlmostEqual(processor.pid.i, 0.002)
        self.assertAlmostEqual(processor.pid.d, 0.012)

    def test_respects_top_crop_boundary(self):
        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        cv.rectangle(frame, (145, 80), (175, 140), (0, 0, 0), -1)
        result = LineProcessor(self.config).process(frame)
        self.assertFalse(result.found)
        self.assertIsNone(result.center_x)

    def test_rejects_invalid_crop(self):
        invalid = dict(self.config, crop=[200, 100])
        with self.assertRaises(ValueError):
            validate_line_config(invalid)

    def test_rejects_invalid_top_crop(self):
        invalid = dict(self.config, crop_top=240)
        with self.assertRaises(ValueError):
            validate_line_config(invalid)


if __name__ == "__main__":
    unittest.main()
