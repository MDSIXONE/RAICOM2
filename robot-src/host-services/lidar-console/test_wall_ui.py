import re
import unittest
from pathlib import Path


HTML_PATH = Path(__file__).resolve().parent / "static" / "index.html"


class WallConfigUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_exposes_every_wall_controller_parameter(self):
        keys = {
            "wall_forward_vx",
            "wall_target_left",
            "wall_stop_front",
            "wall_approach_front",
            "wall_distance_tolerance",
            "wall_heading_tolerance",
            "wall_lateral_kp",
            "wall_min_vy",
            "wall_max_vy",
            "wall_yaw_kp",
            "wall_max_yaw_rate",
            "wall_control_hz",
            "wall_complete_frames",
            "lateral_sign",
        }
        keys.update("wall_second_" + key.removeprefix("wall_") for key in tuple(keys) if key.startswith("wall_") and not key.startswith("wall_second_"))
        keys.update("wall_final_" + key.removeprefix("wall_") for key in tuple(keys) if key.startswith("wall_") and not key.startswith("wall_second_"))
        for key in keys:
            with self.subTest(key=key):
                self.assertIn(key, self.html)

    def test_console_controls_have_unique_element_ids(self):
        ids = re.findall(r'\bid="([^"]+)"', self.html)
        duplicates = sorted({element_id for element_id in ids if ids.count(element_id) > 1})
        self.assertEqual(duplicates, [])

    def test_independent_groups_have_all_thirteen_input_controls(self):
        for prefix in ("wallSecond", "wallFinal"):
            controls = re.findall(r'<input\b[^>]*\bid="(' + prefix + r'[^"]+)"', self.html)
            self.assertEqual(len(controls), 13, prefix)

    def test_wall_sliders_use_realtime_save(self):
        self.assertIn("scheduleWallConfigSave()", self.html)
        self.assertIn("setWallSaveInfo('已实时保存')", self.html)
        self.assertIn("await saveWallConfig(false,false)", self.html)


if __name__ == "__main__":
    unittest.main()
