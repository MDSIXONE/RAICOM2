import asyncio
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))

import server
from server import DEFAULT_CONFIG, compute_wall_command, compute_wall_turn_command


class WallFollowCommandTest(unittest.TestCase):
    def setUp(self):
        self.config = dict(DEFAULT_CONFIG)

    def command(self, front=1.0, left=0.30, current_yaw=10.0, target_yaw=10.0):
        return compute_wall_command(front, left, current_yaw, target_yaw, self.config)

    def test_drives_straight_when_distance_and_heading_are_on_target(self):
        vx, vy, yaw_rate, distance_error, yaw_error, completed = self.command()
        self.assertEqual(vx, 5.0)
        self.assertEqual(vy, 0.0)
        self.assertEqual(yaw_rate, 0.0)
        self.assertEqual(distance_error, 0.0)
        self.assertEqual(yaw_error, 0.0)
        self.assertFalse(completed)

    def test_strafes_left_when_too_far_from_left_wall(self):
        _, vy, _, _, _, _ = self.command(left=0.50)
        self.assertGreater(vy, 0.0)

    def test_strafes_right_when_too_close_to_left_wall(self):
        _, vy, _, _, _, _ = self.command(left=0.20)
        self.assertLess(vy, 0.0)

    def test_heading_error_wraps_across_180_degrees(self):
        _, _, yaw_rate, _, yaw_error, _ = self.command(current_yaw=179.0, target_yaw=-176.0)
        self.assertEqual(yaw_error, 5.0)
        self.assertGreater(yaw_rate, 0.0)

    def test_slows_before_front_stop_distance(self):
        vx, _, _, _, _, completed = self.command(front=0.425)
        self.assertAlmostEqual(vx, 3.0)
        self.assertFalse(completed)

    def test_holds_position_at_front_wall_until_final_alignment(self):
        vx, vy, yaw_rate, _, _, completed = self.command(
            front=0.30,
            left=0.40,
            current_yaw=15.0,
            target_yaw=10.0,
        )
        self.assertEqual(vx, 0.0)
        self.assertGreater(vy, 0.0)
        self.assertLess(yaw_rate, 0.0)
        self.assertFalse(completed)

    def test_completes_only_with_front_left_and_heading_on_target(self):
        vx, vy, yaw_rate, _, _, completed = self.command(front=0.30, left=0.31, current_yaw=11.0)
        self.assertTrue(completed)
        self.assertEqual((vx, vy, yaw_rate), (0.0, 0.0, 0.0))


class WallSequenceTest(unittest.TestCase):
    def setUp(self):
        self.state = server.SharedState()
        self.now = 100.0
        self.state.wall.update(
            enabled=True, phase="straight", segment=1, initial_yaw=10.0,
            target_yaw=10.0, phase_started=99.0,
        )
        self.state.imu.update(connected=True, yaw=10.0, stamp=self.now)
        self.state.lidar.update(connected=True, stamp=self.now)
        self.state.sectors.update(front=1.0, left=0.30)
        self.robot = Mock()
        self.robot.axes.side_effect = lambda vx, vy, yaw_rate, mode: self.state.set_command(vx, vy, yaw_rate, mode)
        self.robot.stop.side_effect = lambda: self.state.set_command(0.0, 0.0, 0.0, "idle")
        self.addCleanup(patch.stopall)
        patch.object(server, "STATE", self.state).start()
        patch.object(server, "ROBOT", self.robot).start()
        self.controller = server.WallFollowController()

    def sample(self, front=0.30, left=0.30, yaw=None, lidar=True, imu=True):
        self.now += 0.1
        self.state.sectors.update(front=front, left=left)
        self.state.imu["yaw"] = self.state.wall["target_yaw"] if yaw is None else yaw
        if lidar:
            self.state.lidar["stamp"] = self.now
        if imu:
            self.state.imu["stamp"] = self.now
        self.controller.step(self.now)

    def confirm_phase(self):
        for _ in range(3):
            self.sample()

    def test_three_straights_two_right_turns_and_no_final_turn(self):
        self.confirm_phase()
        self.assertEqual((self.state.wall["phase"], self.state.wall["segment"]), ("turning", 1))
        self.assertEqual(self.state.wall["target_yaw"], -80.0)
        self.sample(yaw=10.0)
        self.assertEqual(self.robot.axes.call_args.args[:2], (0.0, 0.0))
        self.assertLess(self.robot.axes.call_args.args[2], 0.0)
        self.confirm_phase()
        self.assertEqual((self.state.wall["phase"], self.state.wall["segment"]), ("straight", 2))
        self.sample(front=1.0)
        self.assertGreater(self.state.robot["vx"], 0.0)
        self.assertEqual(self.state.robot["yaw_rate"], 0.0)
        self.confirm_phase()
        self.assertEqual(self.state.wall["target_yaw"], -170.0)
        self.confirm_phase()
        self.assertEqual((self.state.wall["phase"], self.state.wall["segment"]), ("straight", 3))
        self.confirm_phase()
        self.assertFalse(self.state.wall["enabled"])
        self.assertTrue(self.state.wall["completed"])
        self.assertEqual(self.state.wall["phase"], "completed")
        self.assertEqual(self.robot.stop.call_count, 5)
        self.assertEqual((self.state.robot["vx"], self.state.robot["vy"], self.state.robot["yaw_rate"]), (0, 0, 0))
        count = self.robot.axes.call_count
        self.sample()
        self.assertEqual(self.robot.axes.call_count, count)

    def test_right_turn_wraps_negative_180_and_corrects_overshoot(self):
        self.state.wall.update(initial_yaw=-150.0, target_yaw=-150.0)
        self.confirm_phase()
        self.assertEqual(self.state.wall["target_yaw"], 120.0)
        rate, error, reached = compute_wall_turn_command(-150.0, 120.0, self.state.config)
        self.assertLess(rate, 0)
        self.assertEqual(error, -90.0)
        self.assertFalse(reached)
        rate, _, _ = compute_wall_turn_command(116.0, 120.0, self.state.config)
        self.assertGreater(rate, 0)

    def test_confirmation_requires_distinct_sensor_samples(self):
        self.sample()
        for _ in range(3):
            self.sample(lidar=False)
        self.assertEqual(self.state.wall["complete_frames"], 1)
        self.sample()
        self.sample()
        self.assertEqual(self.state.wall["phase"], "turning")
        self.sample()
        self.sample(imu=False)
        self.assertEqual(self.state.wall["complete_frames"], 1)
        self.assertEqual(self.state.wall["phase"], "turning")

    def test_turning_does_not_require_a_left_wall_but_next_leg_requires_a_new_scan(self):
        self.confirm_phase()
        self.sample(left=None, front=None, yaw=10.0)
        self.assertTrue(self.state.wall["enabled"])
        self.assertIsNone(self.state.wall["distance_error"])
        self.confirm_phase()
        self.sample(lidar=False)
        self.assertEqual(self.state.wall["action"], "waiting_scan")
        self.assertEqual(self.state.wall["complete_frames"], 0)
        self.assertEqual(self.robot.axes.call_args.args, (0.0, 0.0, 0.0))
        self.sample(front=1.0)
        self.assertEqual(self.state.wall["action"], "tracking")

    def test_stale_sensors_abort_in_both_phases(self):
        for phase in ("straight", "turning"):
            for sensor in ("imu", "lidar"):
                with self.subTest(phase=phase, sensor=sensor):
                    self.state.wall.update(enabled=True, completed=False, phase=phase, phase_started=self.now)
                    self.state.imu["stamp"] = self.now
                    self.state.lidar["stamp"] = self.now
                    getattr(self.state, sensor)["stamp"] = self.now - 1.0
                    self.controller.step(self.now)
                    self.assertFalse(self.state.wall["enabled"])
                    self.assertEqual(self.state.wall["phase"], "aborted")
                    self.assertIn("数据中断", self.state.wall["error"])

    def test_turn_timeout_and_control_failure_are_visible(self):
        self.state.wall.update(phase="turning", phase_started=self.now - 31.0)
        self.controller.step(self.now)
        self.assertIn("右转 90° 超时", self.state.wall["error"])
        self.state.wall.update(enabled=True, phase="straight", phase_started=99.0)
        self.robot.axes.side_effect = RuntimeError("udp failed")
        self.sample(front=1.0)
        self.assertFalse(self.state.wall["enabled"])
        self.assertIn("udp failed", self.state.wall["error"])

    def test_stop_api_cancels_entire_sequence_without_more_commands(self):
        self.confirm_phase()
        response = asyncio.run(server.api_stop(None))
        self.assertEqual(response.status, 200)
        self.assertFalse(self.state.wall["enabled"])
        self.assertEqual(self.state.wall["phase"], "idle")
        count = self.robot.axes.call_count
        self.sample(yaw=10.0)
        self.assertEqual(self.robot.axes.call_count, count)

    def test_restart_api_begins_at_leg_one_with_a_new_initial_heading(self):
        request = Mock()
        async def body():
            return {"enabled": True}
        request.json = body
        self.state.wall.update(segment=3, phase="turning", completed=True)
        self.state.imu["yaw"] = 75.0
        with patch.object(server.time, "time", return_value=self.now):
            response = asyncio.run(server.api_wall(request))
        result = json.loads(response.text)
        self.assertEqual(result["segment"], 1)
        self.assertEqual(self.state.wall["initial_yaw"], 75.0)
        self.assertEqual(self.state.wall["phase"], "straight")
        self.assertFalse(self.state.wall["completed"])

    def test_third_leg_uses_its_own_speed_target_and_completion_frames(self):
        self.state.config.update(
            wall_final_forward_vx=3.5, wall_final_target_left=0.40,
            wall_final_stop_front=0.45, wall_final_approach_front=0.80,
            wall_final_complete_frames=2.0,
        )
        self.state.wall["segment"] = 3
        self.sample(front=1.0, left=0.40)
        self.assertEqual(self.state.robot["vx"], 3.5)
        self.assertEqual(self.state.robot["vy"], 0.0)
        self.sample(front=0.45, left=0.40)
        self.assertTrue(self.state.wall["enabled"])
        self.sample(front=0.45, left=0.40)
        self.assertTrue(self.state.wall["completed"])
        self.assertEqual(self.state.config["wall_target_left"], 0.30)
        self.assertEqual(self.state.config["wall_forward_vx"], 5.0)

    def test_second_leg_uses_its_own_speed_target_and_completion_frames(self):
        self.state.config.update(
            wall_second_forward_vx=3.5, wall_second_target_left=0.40,
            wall_second_stop_front=0.45, wall_second_approach_front=0.80,
            wall_second_complete_frames=2.0,
        )
        self.state.wall["segment"] = 2
        self.sample(front=1.0, left=0.40)
        self.assertEqual(self.state.robot["vx"], 3.5)
        self.assertEqual(self.state.robot["vy"], 0.0)
        self.sample(front=0.45, left=0.40)
        self.assertTrue(self.state.wall["enabled"])
        self.sample(front=0.45, left=0.40)
        self.assertEqual(self.state.wall["phase"], "turning")
        self.assertEqual(self.state.wall["target_yaw"], -170.0)
        # Turn speed and confirmation must use the common settings.
        self.state.config["wall_second_max_yaw_rate"] = 40.0
        self.sample(yaw=10.0)
        self.assertEqual(self.state.robot["yaw_rate"], -16.0)
        self.sample(yaw=-170.0)
        self.sample(yaw=-170.0)
        self.assertEqual(self.state.wall["phase"], "turning")
        self.sample(yaw=-170.0)
        self.assertEqual((self.state.wall["phase"], self.state.wall["segment"]), ("straight", 3))
        self.assertEqual(self.state.config["wall_target_left"], 0.30)
        self.assertEqual(self.state.config["wall_forward_vx"], 5.0)

    def test_all_final_parameters_are_isolated_from_the_first_two_legs(self):
        config = dict(DEFAULT_CONFIG)
        for index, name in enumerate(server.WALL_PARAMETER_NAMES):
            config["wall_final_" + name] = float(index + 1)
        for segment in (1, 2):
            selected = server.wall_segment_config(config, segment)
            for name in server.WALL_PARAMETER_NAMES:
                self.assertEqual(selected["wall_" + name], config["wall_" + name])
        selected = server.wall_segment_config(config, 3)
        for name in server.WALL_PARAMETER_NAMES:
            self.assertEqual(selected["wall_" + name], config["wall_final_" + name])

    def test_all_second_parameters_are_isolated_from_other_legs(self):
        config = dict(DEFAULT_CONFIG)
        for index, name in enumerate(server.WALL_PARAMETER_NAMES):
            config["wall_second_" + name] = float(index + 1)
        for segment in (1, 3):
            selected = server.wall_segment_config(config, segment)
            for name in server.WALL_PARAMETER_NAMES:
                self.assertEqual(selected["wall_" + name], config["wall_" + name])
        selected = server.wall_segment_config(config, 2)
        for name in server.WALL_PARAMETER_NAMES:
            self.assertEqual(selected["wall_" + name], config["wall_second_" + name])

    def test_old_config_migration_preserves_current_wall_tuning(self):
        previous = {key: value for key, value in DEFAULT_CONFIG.items() if not key.startswith(("wall_final_", "wall_second_"))}
        previous.update(wall_forward_vx=4.0, wall_target_left=0.4, wall_yaw_kp=2.0)
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "config.json"
            config_file.write_text(json.dumps(previous), encoding="utf-8")
            with patch.object(server, "CONFIG_FILE", config_file):
                migrated = server.load_saved_config()
            for name in server.WALL_PARAMETER_NAMES:
                self.assertEqual(migrated["wall_final_" + name], previous["wall_" + name])
                self.assertEqual(migrated["wall_second_" + name], previous["wall_" + name])
            self.assertEqual(json.loads(config_file.read_text(encoding="utf-8")), previous)

    def test_config_api_saves_final_parameters_and_reload_keeps_them(self):
        request = Mock()
        async def body():
            return {"wall_final_forward_vx": 4.0, "wall_final_target_left": 0.42}
        request.json = body
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "config.json"
            with patch.object(server, "CONFIG_FILE", config_file):
                response = asyncio.run(server.api_config(request))
                self.assertEqual(response.status, 200)
                reloaded = server.load_saved_config()
            self.assertEqual(reloaded["wall_final_forward_vx"], 4.0)
            self.assertEqual(reloaded["wall_final_target_left"], 0.42)
            self.assertEqual(reloaded["wall_forward_vx"], 5.0)
            self.assertEqual(reloaded["wall_target_left"], 0.30)

    def test_config_api_saves_second_parameters_and_reload_keeps_them(self):
        request = Mock()
        async def body():
            return {"wall_second_forward_vx": 4.0, "wall_second_target_left": 0.42}
        request.json = body
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "config.json"
            with patch.object(server, "CONFIG_FILE", config_file):
                response = asyncio.run(server.api_config(request))
                self.assertEqual(response.status, 200)
                reloaded = server.load_saved_config()
            self.assertEqual(reloaded["wall_second_forward_vx"], 4.0)
            self.assertEqual(reloaded["wall_second_target_left"], 0.42)
            self.assertEqual(reloaded["wall_forward_vx"], 5.0)
            self.assertEqual(reloaded["wall_target_left"], 0.30)

    def test_invalid_final_parameters_fail_validation_and_config_load(self):
        for change in (
            {"wall_final_forward_vx": 13},
            {"wall_final_approach_front": 0.30},
            {"wall_final_min_vy": 10, "wall_final_max_vy": 5},
            {"wall_final_heading_tolerance": float("nan")},
        ):
            with self.subTest(change=change):
                candidate = {**DEFAULT_CONFIG, **change}
                self.assertIn("第三段", server.validate_config(candidate))
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "config.json"
            config_file.write_text(json.dumps({**DEFAULT_CONFIG, "wall_final_forward_vx": 13}), encoding="utf-8")
            with patch.object(server, "CONFIG_FILE", config_file):
                with self.assertRaisesRegex(ValueError, "第三段配置无效"):
                    server.load_saved_config()
    def test_invalid_second_parameters_fail_validation_and_config_load(self):
        for change in (
            {"wall_second_forward_vx": 13},
            {"wall_second_approach_front": 0.30},
            {"wall_second_min_vy": 10, "wall_second_max_vy": 5},
            {"wall_second_heading_tolerance": float("nan")},
        ):
            with self.subTest(change=change):
                candidate = {**DEFAULT_CONFIG, **change}
                self.assertIn("第二段", server.validate_config(candidate))
        with tempfile.TemporaryDirectory() as directory:
            config_file = Path(directory) / "config.json"
            config_file.write_text(json.dumps({**DEFAULT_CONFIG, "wall_second_forward_vx": 13}), encoding="utf-8")
            with patch.object(server, "CONFIG_FILE", config_file):
                with self.assertRaisesRegex(ValueError, "第二段配置无效"):
                    server.load_saved_config()


if __name__ == "__main__":
    unittest.main()
