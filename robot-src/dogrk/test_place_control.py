"""Placement motion checks without importing hardware."""
import ast
import unittest
from pathlib import Path
from unittest.mock import Mock


class PlaceControlTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).parent / 'lib' / 'place_abcd.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('align_letter_target', 'search_letter_step', 'visible_letter_direction')]
        self.env = {'time': Mock()}
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), self.env)

    def align(self, letter, radius, cx=160):
        dog = Mock()
        complete = self.env['align_letter_target'](
            dog, [cx-radius, 180-radius, cx+radius, 180+radius], 320, letter)
        return dog, complete

    def test_a_releases_farther_than_other_letters(self):
        dog, complete = self.align('A', 22)
        self.assertTrue(complete)
        dog.move_x.assert_not_called()
        self.assertFalse(self.align('A', 21)[1])
        for letter in 'BCD':
            dog, complete = self.align(letter, 20)
            self.assertFalse(complete)
            dog.move_x.assert_called_once_with(3)
            self.assertEqual(self.env['time'].sleep.call_args.args, (.15,))
            self.assertTrue(self.align(letter, 24)[1])

    def test_far_and_near_match_pickup(self):
        for radius, duration in [(12, 1.0), (18, .15), (23, .15)]:
            dog, complete = self.align('D', radius)
            self.assertFalse(complete)
            dog.move_x.assert_called_once_with(3)
            self.assertEqual(self.env['time'].sleep.call_args.args, (duration,))
            dog.stop.assert_called_once()

    def test_off_center_turns_before_approach(self):
        dog, complete = self.align('A', 30, cx=260)
        self.assertFalse(complete)
        dog.turn.assert_called_once_with(-15)
        dog.move_x.assert_not_called()
        self.assertEqual(self.env['time'].sleep.call_args.args, (.74,))

    def test_search_is_lateral_for_one_point_five_seconds(self):
        dog = Mock()
        self.env['search_letter_step'](dog, 0, 8)
        self.assertEqual([c.args for c in dog.move_y.call_args_list], [(8,), (0,)])
        self.assertEqual(self.env['time'].sleep.call_args_list[0].args, (1.5,))
        dog.move_x.assert_not_called()

    def test_seen_b_target_a_moves_left(self):
        detector = Mock()
        detector.detect.side_effect = lambda frame, letter, conf: {'confidence': .9} if letter == 'B' else None
        self.assertEqual(self.env['visible_letter_direction'](detector, None, 'A'), 8)


if __name__ == '__main__':
    unittest.main()
