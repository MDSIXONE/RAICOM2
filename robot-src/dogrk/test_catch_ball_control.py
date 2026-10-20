"""Verify RAICOM reference control without importing hardware drivers."""
import ast
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

class CatchControlTests(unittest.TestCase):
    def run_sequence(self, detections, yaw=-90, start_yaw=0, other_hits=None, lateral=False):
        source = Path(__file__).parent / 'lib' / 'catch_ball.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ('grasp_ball', 'search_ball_step')]
        model = Mock()
        target_results = iter(detections)
        def detect(frame, color, confidence):
            if color == 'red':
                return next(target_results)
            return (other_hits or {}).get((env['capture_frame'].call_count, color))
        model.detect.side_effect = detect
        module = types.ModuleType('delivery_vision')
        module.TargetModel = Mock(return_value=model)
        frame = Mock()
        frame.shape = (240, 320, 3)
        env = {'OPENCV_GREEN_LAB_THRESHOLD': (), 'time': Mock(),
               'capture_frame': Mock(return_value=frame), 'grasp': Mock()}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), env)
        dog, motion = Mock(), Mock()
        dog.read_yaw.return_value = yaw
        motion.start_yaw = start_yaw
        events = []
        env['grasp'].side_effect = lambda _: events.append('grasp')
        motion.change_yaw.side_effect = lambda _: events.append('restore')
        with patch.dict(sys.modules, {'delivery_vision': module}):
            env['grasp_ball'](dog, motion, target_color='red')
        env['grasp'].assert_called_once()
        if not lateral:
            dog.move_y.assert_not_called()
        motion.change_yaw.assert_called_once_with((yaw - start_yaw + 180) % 360 - 180)
        self.assertEqual(events, ['grasp', 'restore'])
        calls = dog.method_calls
        forward = next(i for i, c in enumerate(calls) if c[0] == 'move_x')
        crouch = next(i for i, c in enumerate(calls) if c[0] == 'translation')
        self.assertLess(forward, crouch)
        self.assertEqual(calls[forward].args, (3,))
        self.assertTrue(any(c[0] == 'stop' for c in calls[forward + 1:crouch]))
        self.assertEqual(env['time'].sleep.call_args_list[0].args, (1.3,))
        self.assertEqual(model.detect.call_args.args, (frame, 'red', .60))
        return dog, env

    @staticmethod
    def ball(cx, radius):
        return {'box': [cx-radius, 180-radius, cx+radius, 180+radius], 'confidence': .9}

    def test_right_turn_then_forward_then_grasp(self):
        dog, env = self.run_sequence([self.ball(260, 25)]*3 + [self.ball(160, 20)]*3 + [self.ball(160, 30)]*3)
        dog.turn.assert_called_once_with(-15)
        self.assertEqual([c.args for c in dog.move_x.call_args_list], [(3,), (3,)])
        durations = [c.args[0] for c in env['time'].sleep.call_args_list]
        self.assertIn(.74, durations)
        self.assertIn(.15, durations)

    def test_left_turn(self):
        dog, env = self.run_sequence([self.ball(60, 25)]*3 + [self.ball(160, 30)]*3)
        dog.turn.assert_called_once_with(15)
        dog.move_x.assert_called_once_with(3)

    def test_lost_ball_retries_and_resets_confirmation(self):
        dog, env = self.run_sequence([self.ball(160, 30)]*2 + [None]*8 + [self.ball(160, 30)]*3)
        self.assertEqual(env['capture_frame'].call_count, 13)
        dog.turn.assert_not_called()
        dog.move_x.assert_called_once_with(3)

    def test_repeated_error_does_not_abort(self):
        dog, env = self.run_sequence([self.ball(260, 25)]*30 + [self.ball(160, 30)]*3)
        self.assertEqual(dog.turn.call_count, 10)

    def test_restore_heading_across_imu_wrap(self):
        self.run_sequence([self.ball(160, 30)]*3, yaw=-175, start_yaw=175)

    def test_initial_no_balls_advances_until_target_visible(self):
        dog, env = self.run_sequence([None]*6 + [self.ball(160, 30)]*3)
        self.assertEqual(dog.move_x.call_count, 7)  # One initial approach plus six short steps.
        dog.turn.assert_not_called()

    def test_wrong_color_starts_sweep_and_target_stops_search(self):
        dog, env = self.run_sequence([None]*5 + [self.ball(160, 30)]*3,
                                     other_hits={(1, 'blue'): self.ball(160, 30)}, lateral=True)
        self.assertEqual([c.args[0] for c in dog.move_y.call_args_list], [-8]*4 + [8])
        dog.move_x.assert_called_once_with(3)  # No forward search after seeing another color.
        self.assertEqual(env['capture_frame'].call_count, 8)

    def test_far_then_boundary_then_close_then_grasp(self):
        dog, env = self.run_sequence([self.ball(160, 12)]*3 + [self.ball(160, 18)]*3
                                     + [self.ball(160, 23)]*3 + [self.ball(160, 24)]*3)
        self.assertEqual(dog.move_x.call_count, 4)  # Pre-approach and three visual pulses.
        durations = [c.args[0] for c in env['time'].sleep.call_args_list]
        self.assertEqual(durations, [1.3, .2, 1, 1.0, .15, .15])

    def test_far_off_center_turns_before_forward(self):
        dog, env = self.run_sequence([self.ball(260, 12)]*3 + [self.ball(160, 24)]*3)
        dog.turn.assert_called_once_with(-15)
        dog.move_x.assert_called_once_with(3)  # Only the initial pre-approach.
        self.assertEqual(dog.move_x.call_count, 1)

if __name__ == '__main__':
    unittest.main()
